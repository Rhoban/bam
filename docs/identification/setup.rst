Hardware Setup
==============

BAM's identification pipeline requires a single-axis pendulum test bench. The
actuator drives a pendulum arm; gravity provides a natural, controllable load
that depends on the arm's angle and mass distribution. Recording the resulting
trajectories is enough to identify both the motor model and the friction model.

Pendulum geometry
-----------------

.. code-block:: text

         pivot (actuator)
              |
              | length l
              |
           [mass m]

- The arm is attached directly to the actuator output shaft, with no
  intermediate gearbox or coupling.
- The zero angle is defined as the arm pointing **downward** (stable
  equilibrium). Positive angles are measured counter-clockwise.
- The pendulum can swing freely on both sides up to roughly ±π/2.

The gravity torque at angle :math:`\theta` is:

.. math::

   \tau_e(\theta) = (m + m_\text{arm}/2)\, g\, l\, \sin(\theta)

where :math:`m` is the load mass, :math:`m_\text{arm}` is the arm mass, and
:math:`l` is the arm length. These three values must be measured and passed to
the recording scripts.

Bill of materials
-----------------

- The actuator to identify, mounted rigidly on a fixed frame.
- A pendulum arm (rod or profile) attached to the output shaft.
- A calibrated load mass fixed at the end of the arm.
- A scale to measure :math:`m` and :math:`m_\text{arm}` precisely.
- A ruler or caliper to measure :math:`l` (pivot to center of mass).
- A power supply set to the target voltage.
- A USB-to-serial adapter (Dynamixel / Feetech) or an EtherBan interface (eRob).

Supported actuators
-------------------

The following actuators have bundled recording scripts and pre-identified
parameter files:

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Motor
     - Bus
     - Default Vin
     - Notes
   * - ``mx64``
     - Dynamixel TTL
     - 15 V
     -
   * - ``mx106``
     - Dynamixel TTL
     - 15 V
     -
   * - ``xl320``
     - Dynamixel TTL
     - 7.5 V
     -
   * - ``xl330``
     - Dynamixel TTL
     - 7.5 V
     -
   * - ``erob80_50``
     - EtherBan
     - 48 V
     - Reduction 1:50
   * - ``erob80_100``
     - EtherBan
     - 48 V
     - Reduction 1:100
   * - ``sts3215``
     - Feetech TTL
     - 7.4 V
     -

Mechanical recommendations
--------------------------

- Keep the arm as light and stiff as possible to reduce parasitic modes.
- The load mass should be large enough to excite a range of load torques, but
  small enough that the actuator can move it across the full ±π/2 range.
- Make sure the arm is level at the zero position before recording, to avoid
  a systematic angle offset that would corrupt the gravity term. The fitting
  pipeline includes a ``q_offset`` parameter to absorb small calibration
  errors, but a large offset degrades identifiability.
