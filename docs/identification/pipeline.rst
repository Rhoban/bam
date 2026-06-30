Identification Pipeline
=======================



1. Record raw data
------------------

Record pendulum trajectories using hardware-specific scripts:

- Dynamixel: ``uv run python -m bam.dynamixel.record ...``
- eRob: ``uv run python -m bam.erob.record ...``
- Feetech: ``uv run python -m bam.feetech.record ...``

2. Process logs
---------------

Interpolate raw logs to constant timestep:

.. code-block:: bash

   uv run python -m bam.process --raw data_raw --logdir data_processed --dt 0.005

3. Fit model parameters
-----------------------

Run optimization on processed logs:

.. code-block:: bash

   uv run python -m bam.fit --actuator mx106 --model m6 --logdir data_processed --method cmaes --output params/mx106/m6.json

4. Evaluate and visualize
-------------------------

Compare measured and simulated trajectories:

.. code-block:: bash

   uv run python -m bam.plot --actuator mx106 --logdir data_processed --sim --params params/mx106/m6.json

Supporting modules
------------------

- :mod:`bam.logs`
- :mod:`bam.process`
- :mod:`bam.fit`
- :mod:`bam.simulate`
- :mod:`bam.plot`
