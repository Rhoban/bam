Fitting
=======

The fitting step optimizes the friction model parameters so that the BAM
simulator reproduces the recorded trajectories as closely as possible. The
objective is the mean absolute error (MAE) between simulated and measured
joint positions, averaged across all logs.

Running the fit
---------------

.. code-block:: bash

   python -m bam.fit \
       --actuator xl330 \
       --model m6 \
       --logdir data_processed \
       --output params/xl330/m6.json

``--actuator`` must match the motor name used during recording.
``--model`` selects the friction model variant (``m1`` through ``m6``).

The optimizer writes ``params.json`` every few seconds as it runs, so
progress can be monitored by inspecting the output file or running
``python -m bam.plot`` in parallel.

Optimization options
--------------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Argument
     - Default
     - Description
   * - ``--method``
     - ``cmaes``
     - Optimization algorithm: ``cmaes`` (CMA-ES with BIPOP restart),
       ``random``, or ``nsgaii``.
   * - ``--trials``
     - 100 000
     - Number of evaluations. Increase for better convergence on complex
       models (M5, M6).
   * - ``--workers``
     - 1
     - Number of parallel workers. Uses a shared SQLite study database when
       greater than 1.
   * - ``--load-study``
     - —
     - Path to an existing Optuna study to resume optimization.
   * - ``--reset_period``
     - —
     - Re-synchronize the simulator state to the log at this interval
       [seconds]. Useful when accumulated error destabilizes long rollouts.

Validation split
----------------

To detect overfitting, hold out the logs recorded at one P-gain value and
use them as a validation set:

.. code-block:: bash

   python -m bam.fit \
       --actuator xl330 \
       --model m6 \
       --logdir data_processed \
       --validation_kp 8 \
       --output params/xl330/m6.json

Logs recorded with ``--kp 8`` are excluded from training and evaluated
separately. The best validation MAE is reported alongside the training score.

Fitting all models
------------------

It is recommended to fit all six models and compare their validation error:

.. code-block:: bash

   for model in m1 m2 m3 m4 m5 m6; do
       python -m bam.fit \
           --actuator xl330 \
           --model $model \
           --logdir data_processed \
           --validation_kp 8 \
           --output params/xl330/$model.json \
           --trials 100000
   done

Simpler models (M1, M2) train faster; richer models (M5, M6) may need more
trials to converge but can capture directional and load-dependent effects
that simpler models miss. The model with the best validation MAE and
acceptable parameter count is typically the right choice.

Output file
-----------

The optimizer writes a JSON file containing the identified parameters and
metadata:

.. code-block:: json

   {
     "model": "m6",
     "actuator": "xl330",
     "kt": 2.21,
     "R": 2.03,
     "armature": 0.026,
     "friction_base": 1.0e-05,
     "friction_viscous": 0.051,
     "friction_stribeck": 0.122,
     "dtheta_stribeck": 1.75,
     "alpha": 1.14,
     "load_friction_motor": 0.198,
     "load_friction_external": 0.022,
     "load_friction_motor_stribeck": 0.199,
     "load_friction_external_stribeck": 0.087,
     "load_friction_motor_quad": 0.010,
     "load_friction_external_quad": 7.3e-05,
     "q_offset": 0.0
   }

The file can be passed directly to :func:`bam.model.load_model` or used as
``json_path`` in :class:`bam.mjlab.BamActuatorCfg`.

Evaluating and visualizing results
-----------------------------------

To evaluate a parameter file on the recorded logs:

.. code-block:: bash

   python -m bam.fit \
       --actuator xl330 \
       --model m6 \
       --logdir data_processed \
       --eval \
       --output params/xl330/m6.json

To plot measured versus simulated trajectories:

.. code-block:: bash

   python -m bam.plot \
       --actuator xl330 \
       --logdir data_processed \
       --sim \
       --params params/xl330/m6.json

Several ``--params`` files can be given to overlay multiple models on the
same plot, which is useful for comparing M1 through M6 side by side.

Weights & Biases logging
------------------------

Pass ``--wandb`` to stream training and validation metrics to a W&B project:

.. code-block:: bash

   python -m bam.fit \
       --actuator xl330 \
       --model m6 \
       --logdir data_processed \
       --wandb \
       --output params/xl330/m6.json
