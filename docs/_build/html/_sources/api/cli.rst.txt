CLI and Workflow Modules
========================

These modules are used primarily as command-line entry points.

.. warning::

   Several CLI modules execute argument parsing and runtime code at import time.
   They are documented here as source-oriented workflow references.

Workflow modules
----------------

- ``bam.process``
- ``bam.fit``
- ``bam.plot``
- ``bam.drive_backdrive``
- ``bam.to_mujoco``
- ``bam.dynamixel.record``
- ``bam.dynamixel.all_record``
- ``bam.erob.record``
- ``bam.erob.all_record``
- ``bam.erob.static``
- ``bam.feetech.record``
- ``bam.feetech.all_record``

Example invocation
------------------

.. code-block:: bash

   python -m bam.process --raw data_raw --logdir data_processed --dt 0.005

.. code-block:: bash

   python -m bam.fit --actuator mx106 --model m6 --logdir data_processed --method cmaes
