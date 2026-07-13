Data Acquisition
================

Recording consists of running the actuator through a set of predefined
trajectories while logging position, velocity, and control signals at the
firmware's native rate. The result is a collection of JSON files that are
then resampled to a fixed timestep before fitting.

Installation
------------

To install the dependencies for data acquisition, you need to install the full
project using ``uv`` (`installation instructions <https://docs.astral.sh/uv/getting-started/installation/>`_):

.. code-block:: bash

   uv sync

Available trajectories
----------------------

Each trajectory runs for 6 seconds. The choice of trajectory affects which
friction regimes are excited; using several is recommended.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Name
     - Description
   * - ``sin_time_square``
     - :math:`\sin(t^2)` profile — progressively faster oscillations, good
       general-purpose trajectory that covers a wide velocity range.
   * - ``lift_and_drop``
     - Cubic move to −π/2 over 2 s, then torque disabled — the arm falls
       under gravity. Particularly useful for identifying backdrivability and
       Stribeck effects at very low speed.
   * - ``up_and_down``
     - Cubic path 0 → π/2 → 0.8·π/2 — slower motion, emphasizes static
       friction and load-dependent effects.
   * - ``sin_sin``
     - :math:`\sin(t)\cdot\pi/2 + \sin(5t)\cdot 0.5\cdot\sin(2t)` — rich
       multi-frequency content.
   * - ``nothing``
     - Zero torque command for the full duration — pure gravity response,
       useful to isolate backdrivability.

Recording scripts
-----------------

Dynamixel (MX-64, MX-106, XL-320, XL-330)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   uv run python -m bam.dynamixel.record \
       --port /dev/ttyUSB0 \
       --motor xl330 \
       --mass 0.567 \
       --arm-mass 0.016 \
       --length 0.17 \
       --kp 400 \
       --vin 7.5 \
       --trajectory sin_time_square \
       --logdir data_raw

``--port`` defaults to ``/dev/ttyUSB0``. ``--kp`` and ``--vin`` should match
the values actually programmed in the firmware.

eRob (erob80\_50, erob80\_100)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

eRob actuators are driven through an EtherBan server, which must be running
before recording. The communication relies on generated protobuf bindings,
so first compile the ``.proto`` files:

.. code-block:: bash

   cd bam/erob/
   bash generate_protobuf.sh

You can then monitor the connected devices, which also reports the angular
offset to use for the zero position:

.. code-block:: bash

   uv run python -m bam.erob.etherban

Pass that value to ``--offset`` when recording:

.. code-block:: bash

   uv run python -m bam.erob.record \
       --host 127.0.0.1 \
       --motor erob80_100 \
       --mass 1.2 \
       --arm_mass 0.05 \
       --length 0.25 \
       --offset 0.0 \
       --kp 10.0 \
       --damping 2.0 \
       --trajectory sin_time_square \
       --logdir data_raw

``--offset`` is the angular offset of the zero position in radians, used to
compensate for mounting imprecision.

Feetech (STS3215)
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   uv run python -m bam.feetech.record \
       --port /dev/ttyUSB0 \
       --id 1 \
       --motor sts3215 \
       --mass 0.3 \
       --length 0.12 \
       --kp 32 \
       --vin 7.4 \
       --trajectory sin_time_square \
       --logdir data_raw

Batch recording
~~~~~~~~~~~~~~~

To sweep multiple P-gain values and trajectories in one go:

.. code-block:: bash

   uv run python -m bam.dynamixel.all_record \
       --port /dev/ttyUSB0 \
       --motor xl330 \
       --mass 0.567 \
       --arm-mass 0.016 \
       --length 0.17 \
       --logdir data_raw

This produces several files per combination and is the recommended starting
point for a complete identification run.

Recording strategy
------------------

Vary the P-gain across several values (e.g. 8, 16, 32 for Dynamixel). A
higher gain increases the motor torque during tracking errors, exciting
stronger load-dependent friction. One of these gain values should be set
aside as a **validation set** (see :doc:`fitting`); the others form the
training set.

Record at least two or three different trajectories to cover a broad range
of velocities and load conditions. The ``lift_and_drop`` trajectory is
especially informative for identifying Stribeck and backdrive behavior, and
should always be included.

Raw data format
---------------

Each recording produces one JSON file:

.. code-block:: json

   {
     "mass": 0.567,
     "arm_mass": 0.016,
     "length": 0.17,
     "kp": 400,
     "vin": 7.5,
     "motor": "xl330",
     "trajectory": "sin_time_square",
     "entries": [
       {
         "timestamp": 0.0077,
         "position": 0.0015,
         "speed": 0.024,
         "load": 0.0,
         "input_volts": 7.5,
         "goal_position": 0.0,
         "torque_enable": true
       }
     ]
   }

Entries are logged at the firmware's native rate, which is not necessarily
constant. The processing step resamples them to a fixed timestep.

Checking jitter
---------------

You can use the following script:

.. code-block:: bash

    uv run python -m bam.jitter --logdir data_raw/

To visualize the jitter histogram.

Processing
----------

Resample raw logs to a constant timestep before fitting:

.. code-block:: bash

   uv run python -m bam.process \
       --raw data_raw \
       --logdir data_processed \
       --dt 0.005

``--dt`` is the target timestep in seconds (5 ms is a good default). The
script linearly interpolates between consecutive entries and writes one
processed JSON per raw file into ``data_processed/``.

Plotting
--------

You can plot the processed data using the following command:

.. code-block:: bash

   uv run python -m bam.plot \
       --actuator xl330 \
       --logdir data_processed 