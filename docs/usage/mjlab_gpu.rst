mjlab (MuJoCo GPU)
==================

This page explains how to integrate BAM friction models into an mjlab
pipeline running on GPU with MuJoCo Warp. The entry point is
:class:`bam.mjlab.BamActuatorCfg`.

Installation
------------

BAM is available on PyPI. Install it with the ``mjlab`` extra, which pulls in
mjlab together with mujoco, mujoco-warp and torch:

.. code-block:: text

   pip install better-actuator-models[mjlab]

Or, with `uv <https://docs.astral.sh/uv/>`_:

.. code-block:: text

   uv add "better-actuator-models[mjlab]"

BAM is compatible with mjlab 1.3.

Registering the BAM init event
------------------------------

First, be sure to register the ``bam_init`` function as a startup event in your environment:

.. code-block:: python

    from bam.mjlab import bam_init

    # cfg is your mjlab task configuration
    cfg.events["bam_init"] = EventTermCfg(func=bam_init, mode="startup")



Instantiating the config
------------------------

:class:`~bam.mjlab.BamActuatorCfg` is a dataclass that plugs into mjlab's
actuator system. When an ``Entity`` is built, it instantiates a
:class:`~bam.mjlab.BamActuator` that runs the full BAM pipeline — voltage
control law, DC motor torque, and BAM friction budget — fully vectorized
over all parallel environments via PyTorch tensors.

Two approaches are available, mutually exclusive:

**Bundled motor:**

.. code-block:: python

   from bam.mjlab import BamActuatorCfg

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
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

- Supported bundled motors: see the :doc:`list of identified actuators <actuators>`.
- Supported model variants: ``"m1"`` through ``"m6"`` (see :doc:`../theory/models`).

Voltage and P-gain overrides
-----------------------------

By default, the supply voltage and firmware P-gain are read from the
parameter JSON. They can be overridden at config level:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
      model="m6",
      target_names_expr=(r".*",),
      vin=7.5,      # supply voltage [V]
      kp_fw=125.0,  # firmware P-gain
   )

Domain randomization
--------------------

:class:`~bam.mjlab.BamActuatorCfg` supports per-environment randomization of two
physical quantities that are naturally variable across hardware units or charge
states (battery voltage and supply resistance), plus randomization of the
identified model parameters themselves via their sensitivity ranges.

**Battery voltage** — sample a different supply voltage for each environment
at startup:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
      model="m6",
      target_names_expr=(r".*",),
      vin_range=(7.0, 8.0),   # sampled uniformly at startup [V]
   )

``vin_range`` takes precedence over ``vin`` when both are set.

**Voltage drop resistance** — model battery + cable resistance with a per-env
equivalent resistor between the battery and the motors:

.. math::

   V_\text{eff} = V_\text{in} - R_\text{drop} \, I,
   \qquad
   I = \max\left(0, \; \frac{1}{K_t} \sum_i d_i \, \tau_i \right)

where :math:`R_\text{drop}` (``vin_drop_resistance_range``) is the combined
battery + wire resistance in ohms, :math:`\tau_i` is the actuator torque on
joint :math:`i`, :math:`d_i` its PWM duty cycle, and :math:`K_t` the torque
constant. Randomizing this resistance captures variability in cable length or
connector quality across units:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
      model="m6",
      target_names_expr=(r".*",),
      vin_range=(7.0, 8.0),
      vin_drop_resistance_range=(0.05, 0.15),  # [Ohm] ~100 mOhms of wire & battery resistance
      vin_min=6.0,                             # hard lower bound [V]
   )

Both ranges are sampled once at initialization and held constant across
episode resets.

:math:`I` is the current drawn from the **battery**, not the motor current: an
H-bridge in PWM behaves like a buck stage, so the motor draws
:math:`\tau_i / K_t` continuously while the battery only sources it during the
PWM on-time, giving :math:`I_\text{bat} = d \, I_\text{motor}`. The product is
signed — a joint whose duty cycle and torque disagree in sign is braking and
returning current to the bus — and because all joints of a
:class:`~bam.mjlab.BamActuator` share one supply, the per-joint currents are
summed **before** the :math:`\max(0, \cdot)`. That clamp deliberately discards
regeneration raising the bus voltage, so the model stays conservative during
aggressive decelerations. Both the duty cycle and the torque are taken from the
previous solve, so the drop lags the load by one timestep.

This matches :class:`~bam.mujoco.MujocoController` exactly, so a policy trained
here sees the same battery behaviour as the CPU rollout — see
:doc:`mujoco_cpu` for the same derivation.

.. warning::

   The voltage drop is computed independently by each
   :class:`~bam.mjlab.BamActuator` from its own joints' current draw. If several
   actuator configs share the same physical battery, their currents are **not**
   summed together, so the modeled drop underestimates the real one. Group all
   joints powered by the same battery under a single :class:`~bam.mjlab.BamActuatorCfg`
   if you need the shared-supply behavior. In the current implementation, only one model
   can be used per actuator config, voltage drop will not function properly if different motors
   are mixed in the same config.

**Model parameter randomization** — randomize the identified BAM parameters
themselves (``kt``, ``R``, and the friction terms) per environment, within the
range over which each one only mildly affects the fit. The ranges are read from
a ``<model>_sensitivity.json`` file that must be generated first with
:mod:`bam.sensitivity` — see :ref:`the identification guide <parameter-sensitivity>`.
Enable the randomization with a single flag:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      json_path="params/xl330/m4.json",
      target_names_expr=(r".*",),
      sensitivity_randomization=True,
   )

The flag loads the ``_sensitivity.json`` matching the resolved params path (same
stem, ``_sensitivity.json`` suffix) and, unlike ``vin_range`` /
``vin_drop_resistance_range``, **re-samples the parameters on every environment
reset** — each episode sees a slightly different motor. Sampling is uniform and
**per environment** (one draw shared across all joints of the config, i.e. shape
``(num_envs, 1)``), not per joint. ``armature`` and ``q_offset`` are excluded:
the former is baked into the MuJoCo model at build time, the latter is unused by
the mjlab path. If the sensitivity file is missing, a :class:`FileNotFoundError`
points to :mod:`bam.sensitivity`.

.. note::

   The sensitivity ranges are computed **one parameter at a time**, so
   randomizing all of them jointly compounds: at the default 3% tolerance the
   resulting MAE increase has a median around +7% (with a longer tail), because
   several friction parameters are weakly identified and trade off against each
   other. Regenerate the file at a tighter or looser tolerance to scale the
   randomization band.

Command delay
-------------

BAM inherits mjlab's command delay system, which models the latency between
policy output and motor response (e.g. communication bus latency, firmware
scheduling). The lag is expressed in simulation steps and can be randomized
per environment:

.. code-block:: python

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
      model="m6",
      target_names_expr=(r".*",),
      delay_min_lag=1,    # always at least 1 step of delay
                          # = 5ms with mjlab's default 200Hz control loop
      delay_max_lag=3,    # up to 3 steps, randomized per env
                          # = 15ms with mjlab's default 200Hz control loop
   )

Setting ``delay_min_lag == delay_max_lag`` gives a fixed, deterministic delay.
Leave both at ``0`` (default) to disable delay entirely.

Passing the config to an Entity
--------------------------------

Pass the config to the ``actuator_cfgs`` argument of an mjlab ``Entity``:

.. code-block:: python

   import mjlab

   actuator_cfg = BamActuatorCfg(
      motor_name="{actuator}",
      model="m6",
      target_names_expr=(r".*",),
      kp_fw=125,
      vin_range=(7.0, 8.0),
      vin_drop_resistance_range=(0.05, 0.15),  # [Ohm]
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
