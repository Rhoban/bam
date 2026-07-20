MuJoCo (CPU)
============

This page explains how to plug BAM friction models into a standard MuJoCo
simulation running on CPU. The entry point is :class:`bam.mujoco.MujocoController`.

Installation
------------

BAM is available on PyPI. Install it with the ``mujoco`` extra to pull in the
MuJoCo dependency:

.. code-block:: text

   pip install better-actuator-models[mujoco]

Or, with `uv <https://docs.astral.sh/uv/>`_:

.. code-block:: text

   uv add "better-actuator-models[mujoco]"

Overview
--------

At each simulation step, :class:`~bam.mujoco.MujocoController` does three things:

1. Optionally lowers the supply voltage by a drop proportional to the previous
   step's load, to model battery + cable resistance.
2. Computes the motor torque from a firmware-like P-controller — optionally
   clipping it to the firmware current limit — and applies it via
   ``mj_data.ctrl``.
3. Evaluates the BAM friction model and writes the result into
   ``mj_model.dof_frictionloss`` and ``mj_model.dof_damping``.

Loading a model
---------------

Use :func:`bam.model.load_model` to obtain a :class:`~bam.model.Model` object.
Two approaches are available.

**Bundled motor** — the library ships identified parameters for a set of common
servos:

.. code-block:: python

   from bam.model import load_model

   model = load_model(motor_name="{actuator}", model="m6")

- Supported motor names: see the :doc:`list of identified actuators <actuators>`.
- Supported model variants: ``"m1"`` through ``"m6"`` (see :doc:`../theory/models`).

**Custom JSON** — parameters produced by your own identification run:

.. code-block:: python

   model = load_model("path/to/params.json")

XML setup
---------

Each actuator must be declared as a ``motor`` in the MJCF file (not
``position`` or ``velocity``). BAM overwrites ``frictionloss``, ``damping``,
and ``armature`` at runtime, so any value set in the XML will be ignored.

.. code-block:: xml

   <actuator>
     <motor name="joint_1" joint="joint_1" gear="1"/>
     ...
     <motor name="joint_n" joint="joint_n" gear="1"/>
   </actuator>


Instantiating the controller
-----------------------------

.. code-block:: python

   import mujoco
   from bam.mujoco import MujocoController

   mj_model = mujoco.MjModel.from_xml_file("robot.xml")
   mj_data  = mujoco.MjData(mj_model)

   controller = MujocoController(
      model=model,
      actuator=["joint_1", ..., "joint_n"],  # must match the motor name in the XML
      mujoco_model=mj_model,
      mujoco_data=mj_data,
   )

The ``actuator`` argument can take a single string or a list of strings, which 
allows the same motor model to drive multiple joints. Each string must 
match the ``name`` attribute of the ``<motor>``.

Simulation loop
---------------

Inside the loop, call :meth:`~bam.mujoco.MujocoController.set_q_target` to
provide the desired joint angle, then :meth:`~bam.mujoco.MujocoController.update`
before every ``mj_step``:

.. code-block:: python

   mujoco.mj_resetData(mj_model, mj_data)

   joint_names = ["joint_1", ..., "joint_n"]
   target_angles = [...]

   while True:
      for joint_name, target_angle in zip(joint_names, target_angles):
         controller.set_q_target(joint_name, target_angle)
      controller.update()
      mujoco.mj_step(mj_model, mj_data)

Voltage drop (optional)
-----------------------

Real batteries and cables have an internal resistance that causes a voltage
drop under load. BAM models this as an equivalent resistor placed between the
battery and the motors:

.. math::

   V_\text{eff} = V_\text{in} - R_\text{drop} \, I,
   \qquad
   I = \max\left(0, \; \frac{1}{K_t} \sum_i d_i \, \tau_i \right)

where ``vin_drop_resistance`` is :math:`R_\text{drop}` (the combined battery +
wire resistance, in ohms), :math:`\tau_i` is the actuator torque on joint
:math:`i`, :math:`d_i` its PWM duty cycle, and :math:`K_t` the torque constant.
A hard lower bound ``vin_min`` can be set to prevent the effective voltage from
collapsing under heavy load:

.. code-block:: python

   controller = MujocoController(
      model=model,
      actuator=["joint_1", ..., "joint_n"],
      mujoco_model=mj_model,
      mujoco_data=mj_data,
      vin_drop_resistance=0.1,   # 100 mOhms of wire & battery resistance
      vin_min=6.0,               # [V]
   )

Note that :math:`I` is the current drawn from the **battery**, which is not the
motor current. An H-bridge in PWM behaves like a buck stage: the motor draws
:math:`\tau_i / K_t` continuously, but the battery only sources it during the
on-time of the PWM period, so the two are related by the duty cycle,

.. math::

   I_\text{bat} = d \, I_\text{motor},
   \qquad
   V_\text{bat} I_\text{bat} = V_\text{motor} I_\text{motor}.

Two consequences are worth keeping in mind when fitting
``vin_drop_resistance``:

* **The product is signed.** When :math:`d_i` and :math:`\tau_i` have opposite
  signs the joint is braking and pushing current *back* into the bus. Since all
  joints of a controller share one supply, the per-joint currents are summed
  **before** the clamp, so a regenerating joint offsets a driving one.
* **Regeneration cannot raise the voltage.** The :math:`\max(0, \cdot)` discards
  a net negative draw rather than modelling the bus voltage rising above
  :math:`V_\text{in}`. This is deliberate — most drives dissipate that energy
  instead of returning it to the pack — but it means the model is conservative
  during aggressive decelerations.

Both quantities are taken from the previous simulation step (the duty cycle from
the last :meth:`~bam.mujoco.MujocoController.update` and the torque from the
last solve), so the drop lags the load by one timestep and no algebraic loop is
introduced.

.. warning::

   The voltage drop is computed independently by each
   :class:`~bam.mujoco.MujocoController` from its own joints' current draw.
   If several controllers share the same physical battery, their currents are
   **not** summed together, so the modeled drop underestimates the real one.
   Group all joints powered by the same battery under a single controller if you
   need the shared-supply behavior.

Multi-actuator config file
--------------------------

For robots with many joints, :func:`bam.mujoco.load_config` loads a JSON
configuration file that maps each group of joints to a model:

.. code-block:: python

   from bam.mujoco import load_config

   controllers, dof_to_controller = load_config(
      path="config.json",
      mujoco_model=mj_model,
      mujoco_data=mj_data,
      kp=125.0,
      vin=7.5,
   )

The config file has the following structure:

.. code-block:: json

   {
      "arm": {
         "dofs": ["shoulder", "elbow"],
         "model": {
            "kt": 1.6224667906987444,
            "R": 3.949433673232461,
            "armature": 0.011951238325312509,
            "friction_base": 0.09038677246291783,
            "friction_viscous": 0.011691602145974832,
            "model": "m1",
            "actuator": "mx64"
         },
         "error_gain": 1.0,
         "max_pwm": 885
      },
      "leg": {
         "dofs": ["hip", "knee", "ankle"],
         "model": {
            "kt": 2.1913757006745245,
            "R": 2.9649903987776804,
            "armature": 0.026609234235148084,
            "friction_base": 0.10352026623606064,
            "friction_viscous": 0.03520238029013507,
            "model": "m1",
            "actuator": "mx106"
         },
         "error_gain": 1.0,
         "max_pwm": 885
      }
   }

``controllers`` is a dict keyed by group name; ``dof_to_controller`` maps each
DOF name back to its group.

API reference
-------------

- :class:`bam.mujoco.MujocoController`
- :func:`bam.mujoco.load_config`
- :func:`bam.model.load_model`
