Contributing a Model
====================

If you have identified parameters for a motor that is not yet in the BAM
library, sharing them benefits the whole community. This page explains how
to format and submit your contribution.

What to include
---------------

A contribution consists of:

- One JSON parameter file per model variant (at minimum M6; ideally M1–M6).
- The motor name and a short description (bus type, reduction ratio if any,
  nominal voltage).
- The pendulum configuration used for identification (mass, arm mass, length)
  and the recording conditions (voltage, P-gain values, trajectories).

Parameter file format
---------------------

The JSON file produced by ``python -m bam.fit`` is the expected format:

.. code-block:: json

   {
     "model": "m6",
     "actuator": "my_motor",
     "kt": 1.23,
     "R": 4.56,
     "armature": 0.01,
     "friction_base": 0.05,
     "friction_viscous": 0.02,
     "friction_stribeck": 0.08,
     "dtheta_stribeck": 1.5,
     "alpha": 1.2,
     "load_friction_motor": 0.15,
     "load_friction_external": 0.03,
     "load_friction_motor_stribeck": 0.10,
     "load_friction_external_stribeck": 0.04,
     "load_friction_motor_quad": 0.005,
     "load_friction_external_quad": 0.001,
     "q_offset": 0.0
   }

The file is generated automatically at the end of a fitting run. No manual
editing is required.

Quality checklist
-----------------

Before submitting, verify:

- The identified parameters reproduce recorded trajectories with a reasonable
  MAE (use ``python -m bam.plot --sim`` to inspect the fit visually).
- The fit was validated on a hold-out set (``--validation_kp``) to check for
  overfitting.
- All six models (M1–M6) have been fitted so users can choose the complexity
  level appropriate for their use case.

Submitting
----------

Open a pull request on the `BAM GitHub repository <https://github.com/Rhoban/bam>`_
and place your parameter files under:

.. code-block:: text

   bam/params/<motor_name>/m1.json
   bam/params/<motor_name>/m2.json
   ...
   bam/params/<motor_name>/m6.json

Include a short description of the motor and the identification conditions in
the pull request body.
