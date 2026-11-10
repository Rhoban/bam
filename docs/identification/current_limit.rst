Servos with a firmware current limit
====================================

Some servo firmwares limit the motor current while in **position** control. BAM
models this as a constraint on the PWM duty cycle, since the duty is all the
firmware can act on. This page describes how such a servo is recorded, how the
limit is checked against the real servo, and how it is fitted, using the
**Dynamixel XL330-M288-T with metal gears** (``xl330m288t-MG``) as the example.

.. note::

   ``xl330m288t-MG`` runs ROBOTIS's custom "impact guard" firmware **V4**, which
   enforces the ``Current Limit`` register in position control (the factory
   firmware only does so in the current-based operating modes). It is
   configured as on the `microduck <https://github.com/pollen-robotics>`_ robot:
   1000 mA limit, impact guard off.

Supply-side current limit
-------------------------

The XL330 senses the current on the **supply** side of the H-bridge, not in the
motor phase. By power balance the reported current is

.. math::

   I_\text{supply} = |\text{duty}| \cdot I_\text{phase},
   \qquad I_\text{phase} = \frac{\text{duty}\, v_\text{in} - k_t \dot q}{R}

Keeping :math:`|I_\text{supply}| \le I_\text{lim}` bounds the duty cycle to the
roots of :math:`v_\text{in}\,\text{duty}^2 - k_t \dot q\,\text{duty} - R\,I_\text{lim} \le 0`:

.. math::

   \frac{k_t \dot q - S}{2 v_\text{in}} \le \text{duty} \le \frac{k_t \dot q + S}{2 v_\text{in}},
   \qquad S = \sqrt{(k_t \dot q)^2 + 4\, v_\text{in} R\, I_\text{lim}}

This is :class:`~bam.dynamixel.actuator.XL330SupplyCurrentLimitedActuator`. The
P-law duty is clamped to this window, then to ``[-max_pwm, max_pwm]``. Unlike a
phase-current limit (the ``max_current`` option of
:class:`~bam.actuator.VoltageControlledActuator`), the allowed duty does not
vanish at stall: it grows with the square root of the limit (≈ 0.6 at stall for
1 A on the XL330). The limit is a model parameter, ``current_limit`` [A].

Recording
---------

For ``--motor xl330m288t-MG``, ``bam.dynamixel.record`` checks and configures the
servo before every recording, so the bench always matches the robot:

* it refuses to run unless the servo reports ``firmware_version`` 4;
* with torque off, it writes 0 to *Impact Guard Enable* (register 67, RAM, off at
  power-up) and reads it back;
* it writes and reads back ``position_i_gain`` = 0, ``position_d_gain`` = 0
  (the factory D gain is not zero), ``pwm_slope`` = 255, ``current_limit`` =
  1000 mA and ``shutdown`` = 52.

``shutdown`` = 52 clears the input-voltage bit: a 2S supply sits above the
default 7.0 V *Max Voltage Limit*, and with the factory value (53, which a
firmware update restores) the input-voltage error latches torque off. A fault
that already latched needs a reboot of the servo
(``Xl330PyController.reboot``) before torque can be enabled again.

The state is read in one 12-byte transaction (*Present PWM*, *Present Current*,
*Present Velocity*, *Present Position*, registers 124–135), about 1.7× the
previous sample rate, and every entry also logs the supply current, ``current``
[A]. It is not used by the fit, but it is what the limit is checked against.

Trajectories
~~~~~~~~~~~~

The four standard trajectories barely reach a 1 A limit (0.1 % of the samples).
For this motor ``all_record`` also plays ``steps`` and ``big_steps``:
``big_steps`` holds a sequence of ±0.4 to ±1.4 rad steps for 0.5 s each, driving
the servo into its limit at every step and to velocities (up to ~15 rad/s) the
smooth trajectories never reach. A full session is therefore 5 P-gains × 6
trajectories = 30 recordings per mass.

.. code-block:: text

   uv run python -m bam.dynamixel.all_record \
       --port /dev/ttyACM0 \
       --motor xl330m288t-MG \
       --mass 0.10487 \
       --arm-mass 0.02327 \
       --length 0.1 \
       --vin 7.4 \
       --logdir data_raw

Checking the limit against the servo
------------------------------------

Before fitting, the logged duty and current tell what the firmware does, without
any mechanical model. On the XL330-M288-T with V4 firmware:

* the limit regulates the **supply** current at the register value: on a step,
  the current overshoots to ≈ 1.25 A for ~4 ms, then holds at 0.95–1.0 A while
  the duty is pulled down from the commanded value;
* the duty at the limit is ≈ 0.82 at stall and rises with velocity, reaching
  full duty around 5 rad/s, where back-EMF keeps the current below the limit;
* the limit is only active for 20–30 ms at the onset of a large step;
* ``pwm_slope`` = 255 settles the duty in 1–2 ms, below BAM's 5 ms step, so it
  needs no model.

The stall current against duty also shows the supply-side relation: across the
stall bins, the supply-side model gives a fairly stable resistance (3.4–5 Ω)
where a phase-side one drifts from 6 to 12 Ω.

Fitting
-------

Fit with ``current_limit`` **fixed** to the register value:

.. code-block:: text

   uv run python -m bam.fit \
       --actuator xl330m288t-MG \
       --model m6 \
       --logdir data_processed \
       --set '{"current_limit": 1.0}' \
       --trials 3000 --workers 1

Left free, ``current_limit`` is poorly constrained by the standard trajectories
(fits landed anywhere from 1.1 to 2.9 A) and the model then behaves as without a
limit on large steps. Fixed at 1.0 A it gave the best held-out error, and the
best free fits did land close to it (1.09 and 1.18 A).

.. tip::

   ``--trials`` is per worker: ``--workers 6 --trials 20000`` runs 120 000
   trials. With 17 parameters (m6) single runs also end in different local
   minima (25–134 mrad on the same data here), so several independent
   ``--workers 1`` runs of ~3000 trials, keeping the best, are faster and
   more reliable than one long multi-worker run.

Results: XL330-M288-T metal gears
---------------------------------

Bench: 0.1 m arm, effective arm mass 0.02327 kg (25.86 g with its centre of mass
at 4.5 cm, scaled so that the gravity torque matches the uniform-rod model), tip
masses 21.21 / 63.04 / 104.87 g, P-gains 50–400, 6 trajectories: 90 logs, with
all the kp 200 logs held out for validation.

.. list-table:: m6, position MAE on the held-out kp 200 logs [mrad]
   :header-rows: 1

   * - Model
     - all held-out logs
     - ``big_steps``
   * - ``current_limit`` fixed at 1.0 A
     - **14.3**
     - **16–20**
   * - no current limit
     - 16.2–17.4
     - 28–36

Without the limit, the fit also doubles ``command_delay`` (10 ms instead of
5 ms) to compensate for the torque it gets wrong at step onsets.

Compared with the plastic-gear XL330 (``xl330``), kt, R and the armature are
within a few percent; the difference is in friction, mainly the load-dependent
Stribeck term (``load_friction_motor_stribeck`` ≈ 0.23 vs ≈ 0).

Known limitations
~~~~~~~~~~~~~~~~~

* Only **m6** is identified for ``xl330m288t-MG`` so far.
* While the limit is active, the simulated duty is 0.10–0.19 below the measured
  one, although position and torque match. The stall current follows
  :math:`I \propto (\text{duty} + 0.074)^2`: the motor behaves as if it got ~7 %
  more duty than the servo reports, which BAM's linear electrical model cannot
  represent (it compensates with a lower effective R). A duty offset parameter
  would likely fix it, if the duty itself matters.
* The raw logs are not published yet.
