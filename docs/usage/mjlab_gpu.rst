Using BAM in MuJoCo Warp via mjlab (GPU)
=========================================

This page explains how to integrate BAM friction models into an mjlab
pipeline running on GPU with MuJoCo Warp. The entry point is
:class:`bam.mjlab.BamActuatorCfg`.

Installation
------------

BAM is available on PyPI. Install it with the ``mjlab`` extra, which pulls in
mjlab together with mujoco, mujoco-warp and torch:

.. code-block:: text

   pip install better-actuator-models[mjlab]

BAM is compatible with mjlab 1.3.


Overview
--------

:class:`~bam.mjlab.BamActuatorCfg` is a dataclass that plugs into mjlab's
actuator system. When an ``Entity`` is built, it instantiates a
:class:`~bam.mjlab.BamActuator` that runs the full BAM pipeline — voltage
control law, DC motor torque, and BAM friction budget — fully vectorized
over all parallel environments via PyTorch tensors.

Instantiating the config
-------------------------

Two approaches are available, mutually exclusive:

**Bundled motor:**

.. code-block:: python

   from bam.mjlab import BamActuatorCfg

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
   )

**Custom JSON** (parameters produced by ``bam.fit``):

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      json_path="params/my_motor/m6.json",
      target_names_expr=(r".*",),
   )

The ``target_names_expr`` field is a tuple of regex patterns that select
which actuated joints this config controls.

Supported bundled motors: ``"xl330"``, ``"xl320"``, ``"mx106"``, ``"mx64"``,
``"erob80:50"``, ``"erob80:100"``.
Supported model variants: ``"m1"`` through ``"m6"`` (see :doc:`../theory/models`).

Voltage and P-gain overrides
-----------------------------

By default, the supply voltage and firmware P-gain are read from the
parameter JSON. They can be overridden at config level:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
      vin=7.5,      # supply voltage [V]
      kp_fw=125.0,  # firmware P-gain
   )

Domain randomization
--------------------

:class:`~bam.mjlab.BamActuatorCfg` supports per-environment randomization
of two physical quantities that are naturally variable across hardware units
or charge states.

**Battery voltage** — sample a different supply voltage for each environment
at startup:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
      vin_range=(7.0, 8.0),   # sampled uniformly at startup [V]
   )

``vin_range`` takes precedence over ``vin`` when both are set.

**Voltage drop gain** — model battery + cable resistance with a per-env
internal-resistance gain:

.. math::

   V_\text{eff} = V_\text{in} - g_\text{drop} \sum_i |\tau_i|

where :math:`g_\text{drop} \approx R / K_t`. Randomizing this gain captures
variability in cable length or connector quality across units:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
      vin_range=(7.0, 8.0),
      vin_drop_gain_range=(0.3, 0.7),  # [V/Nm]
      vin_min=6.0,                     # hard lower bound [V]
   )

Both ranges are sampled once at initialization and held constant across
episode resets.

Current limiting
----------------

Servo firmwares try to cap the motor current to protect the hardware. The firmware
can only act on the PWM duty cycle, though — it cannot synthesize a voltage the
battery does not have — so a current limit is really a *duty-cycle constraint*, not
a hard clamp on the output torque. BAM models it that way in
:meth:`~bam.actuator.VoltageControlledActuator.compute_control`: from the motor
relation :math:`I = (\texttt{duty}\cdot V_\text{in} - K_t\dot{q}) / R`, the
constraint :math:`|I| \le \texttt{max\_current}` maps to a duty window that is
clamped and then intersected with the physical ``[-max_pwm, max_pwm]`` range,
applied **last**. At high speed the back-EMF :math:`K_t\dot{q}` can push the window
outside the achievable PWM range, in which case the limiter saturates and the
current is *not* actually held at ``max_current`` — exactly as the real firmware
behaves when the battery cannot supply the required voltage.

``max_current`` is a property of the actuator model (``VoltageControlledActuator``),
not a field of ``BamActuatorCfg``. It is set per motor family — for instance the
``xl330`` uses ``max_current = 1.75`` A — so the limiter is applied automatically
based on the selected ``motor_name``. Actuators whose ``max_current`` is ``None``
(default) perform no current limiting.

Command delay
-------------

BAM inherits mjlab's command delay system, which models the latency between
policy output and motor response (e.g. communication bus latency, firmware
scheduling). The lag is expressed in simulation steps and can be randomized
per environment:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
      delay_min_lag=1,    # always at least 1 step of delay
      delay_max_lag=3,    # up to 3 steps, randomized per env
   )

Setting ``delay_min_lag == delay_max_lag`` gives a fixed, deterministic delay.
Leave both at ``0`` (default) to disable delay entirely.

Passing the config to an Entity
--------------------------------

Pass the config to the ``actuator_cfgs`` argument of an mjlab ``Entity``:

.. code-block:: python

   import mjlab

   actuator_cfg = BamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=(r".*",),
      kp_fw=125,
      vin_range=(7.0, 8.0),
      vin_drop_gain_range=(0.3, 0.7),
      vin_min=6.0,
      delay_min_lag=1,
      delay_max_lag=3,
   )

   entity = mjlab.Entity(
      xml_path="robot.xml",
      actuator_cfgs=(actuator_cfg,),
   )

mjlab calls :meth:`~bam.mjlab.BamActuatorCfg.build` internally to create the
:class:`~bam.mjlab.BamActuator` and wire it into the simulation graph.

API reference
-------------

- :class:`bam.mjlab.BamActuatorCfg`
- :class:`bam.mjlab.BamActuator`
- :func:`bam.model.load_model`
