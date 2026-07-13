Hardware Setup
==============

BAM's identification pipeline requires a pendulum test bench with variable 
loads and lengths to increase the variety of the collected logs. The pendulum 
is attached directly to the actuator's output shaft, optionally using a 
counter-shaft when the actuator supports it.

  To present the identification process, the Dynamixel XL-330 motor is used as 
  a case study. The instructions are similar for other actuators, adjusting only 
  for the mass and length parameters.

Requirements
------------

The hardware required for the identification process is as follows:

- **A set of rigid arms of varying lengths**.

  The mass of the arms should be negligible compared to the attached loads.
  Therefore, it is recommended to use 3D-printed or laser-cut wooden arms for
  smaller motors handling light loads, and to reserve metal arms for more
  powerful motors.

- **A set of weights compatible with the arms**.

  The masses must be heavy enough to generate a wide range of load torques.

- **A mounting bracket for the actuator**.

  The setup must ensure that the actuator remains firmly secured during data
  logging, despite the fast movements of the weights at the end of the arm. The
  arm-weight assemblies must have sufficient clearance to oscillate between
  +/- 90° relative to the vertical position.

- **A communication interface for the actuator**.

Once the hardware is gathered, you must record the mass of the weights as well 
as the length and mass of the arms. These parameters are mandatory for the identification process.

Example: Dynamixel XL-330
-------------------------

Here is an example of a test bench for the Dynamixel XL-330 actuator. 
The pendulum arms are 3D printed; you can refer to the 3D model for inspiration 
`here <https://cad.onshape.com/documents/c132b33797dc72aa58be8a7c/v/d9921c19039fee61b5e51bba/e/8e6b266057c61625732fd9b0?renderMode=0&uiState=6a447ab873e2fce629279db5>`_.

.. image:: /_static/hardware_1.png
   :width: 80%
   :align: center
   :alt: 3D-printed pendulum test bench for the Dynamixel XL-330

The interface with the actuator is established using a U2D2 kit.

.. image:: /_static/hardware_2.png
   :width: 80%
   :align: center
   :alt: U2D2 communication interface

The complete setup is shown below. The pendulum is attached to the
actuator's output shaft, and the masses are fixed to the end of the
pendulum arm.

.. image:: /_static/hardware_3.png
   :width: 80%
   :align: center
   :alt: Complete Dynamixel XL-330 test bench setup