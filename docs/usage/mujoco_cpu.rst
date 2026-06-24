Using BAM in MuJoCo (CPU)
=========================

This page explains how to plug BAM friction models into a standard MuJoCo
simulation running on CPU. The entry point is :class:`bam.mujoco.MujocoController`.

Installation
------------

BAM is available on PyPI:

.. code-block:: bash

   pip install bam

Overview
--------

At each simulation step, :class:`~bam.mujoco.MujocoController` does three things:

1. Computes the motor torque from a firmware-like P-controller and applies it
   via ``mj_data.ctrl``.
2. Evaluates the BAM friction model and writes the result into
   ``mj_model.dof_frictionloss`` and ``mj_model.dof_damping``.
3. Optionally applies a voltage drop proportional to the current load, to
   model battery + cable resistance.

Loading a model
---------------

Use :func:`bam.model.load_model` to obtain a :class:`~bam.model.Model` object.
Two approaches are available.

**Bundled motor** — the library ships identified parameters for a set of common
servos:

.. code-block:: python

   from bam.model import load_model

   model = load_model(motor_name="xl330", model="m6")

Supported motor names: ``"xl320"``, ``"xl330"``, ``"xl330i"``, ``"mx106"``, ``"mx64"``,
``"erob80:50"``, ``"erob80:100"``.
Supported model variants: ``"m1"`` through ``"m6"`` (see :doc:`../theory/models`).

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
   controller.reset(mj_data.qpos)

   joint_names = ["joint_1", ..., "joint_n"]
   target_angles = [...]

   while True:
      for joint_name, target_angle in zip(joint_names, target_angles):
         controller.set_q_target(joint_name, target_angle)
      controller.update()
      mujoco.mj_step(mj_model, mj_data)

:meth:`~bam.mujoco.MujocoController.reset` should be called after every
``mj_resetData`` to clear the internal velocity and torque state.

Voltage drop (optional)
-----------------------

Real batteries and cables introduce a voltage drop proportional to the total
current draw. BAM models this as:

.. math::

   V_\text{eff} = V_\text{in} - g_\text{drop} \sum_i |\tau_i|

where :math:`g_\text{drop}` is ``vin_drop_gain`` (approximately
:math:`R / K_t`) and the sum runs over all controlled joints.
A hard lower bound ``vin_min`` can be set to prevent the effective voltage
from collapsing under heavy load:

.. code-block:: python

   controller = MujocoController(
      model=model,
      actuator=["joint_1", ..., "joint_n"],
      mujoco_model=mj_model,
      mujoco_data=mj_data,
      vin_drop_gain=0.5,   # [V/Nm]
      vin_min=6.0,         # [V]
   )

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
         "model": { ... },
         "error_gain": 1.0,
         "max_pwm": 885
      },
      "leg": {
         "dofs": ["hip", "knee", "ankle"],
         "model": { ... },
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
