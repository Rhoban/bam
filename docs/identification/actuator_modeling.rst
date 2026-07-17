Modeling the actuator
=====================

Before you start getting some data, you need to model your actuator in BAM software. Modeling an actuator consist
in subclassing BAM's :class:`~bam.actuator.Actuator` class and implementing the required methods. The following
questions need to be addressed:

* How is the servomotor turning the state (encoder position, velocity) into **torque** ?
    * What **control law** is in play ?
    * What **limits** or **smoothing** is applied in the servo firmware ?
* What important **values** need to be identified to model the actuator ? 
    * In particular, you might want to have some initial guess ranges for those values

Fortunately, the answers to those questions are often the same. That is why BAM already provides few
subclasses of :class:`~bam.actuator.Actuator`:

* :class:`~bam.actuator.VoltageControlledActuator` for actuators that are controlled by a voltage command
* :class:`~bam.actuator.CurrentControlledActuator` for actuators that are controlled by a current command 

Voltage controlled actuator
---------------------------

If the command produced in firmware is a voltage, then you should subclass :class:`~bam.actuator.VoltageControlledActuator`. Find examples of such subclassing in `Dynamixel's actuator.py <https://github.com/Rhoban/bam/blob/main/bam/dynamixel/actuator.py>`_.

Here are the constructor parameters:

* ``vin``: the supply voltage for the actuator
* ``kp``: the proportional gain of the firmware control law, we recommend keeping it in *firmware unit*, meaning that you can use the exact same value as the one documented by the vendor
* ``error_gain``: this gain should be such that ``u = error_gain * kp * (target - position)`` is the voltage command sent to the motor. It is a constant that correct the firmware unit into a voltage unit
* ``max_pwm``: the maximum PWM value (between ``0`` and ``1``) that the firmware can set the PWM to. If you don't know, you can set it to ``1``.
* ``max_current``: if the firmware has a current limit, you can set it here. If you don't know, you can set it to ``None``.

And here are the typical model parameters you'll want to identify:

* :math:`k_t`: the torque constant of the motor, in :math:`N.m/A`
* :math:`R`: the resistance of the motor, in :math:`\Omega`
* :math:`I`: the apparent inertia of the motor, in :math:`kg.m^2`

See below for initial guesses and ranges for those parameters.

Current controlled actuator
---------------------------

If the command produced in firmware is a current, then you should subclass :class:`~bam.actuator.CurrentControlledActuator`. Find examples of such subclassing in `Dynamixel's actuator.py <https://github.com/Rhoban/bam/blob/main/bam/dynamixel/actuator.py>`_.

Here are the  constructor parameters:

* ``vin``: the supply voltage for the actuator
* ``kp``: the proportional gain of the firmware control law, we recommend keeping it in *firmware unit*, meaning that you can use the exact same value as the one documented by the vendor
* ``error_gain``: this gain should be such that ``i = error_gain * kp * (target - position)`` is the current command sent to the motor. 

And here are the typical model parameters you'll want to identify:

* :math:`k_t`: the torque constant of the motor, in :math:`N.m/A`
* :math:`R`: the resistance of the motor, in :math:`\Omega`
* :math:`I`: the apparent inertia of the motor, in :math:`kg.m^2`
* :math:`I_{max}`: the maximum current that the firmware can send to the motor, in :math:`A`

Initial guesses and ranges
---------------------------

When declaring a model, you need to provide initial guesses and ranges for the parameters. For example:

.. code-block:: python

   # Torque constant [Nm/A] or [V/(rad/s)]
   self.model.kt = Parameter(0.7, 0.25, 1.5)

This  tells BAM that the initial guess for :math:`k_t` is ``0.7``, and that it should be searched in the range ``[0.25, 1.5]``.

Torque constant :math:`k_t`
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Torque constant :math:`k_t` maps current in :math:`A` to torque in :math:`N.m`. It turns out that the same constant
also maps the motor velocity in :math:`rad/s` to back-EMF voltage in :math:`V`.

Thus, to estimate :math:`k_t`, you can apply a known voltage to the motor and measure its free speed :math:`\omega`. The guess for :math:`k_t` is then given by :math:`k_t = \frac{V}{\omega}`. 

Don't hesitate to take multiple samples and use a method like least square to estimate the torque constant more accurately.


Resistance :math:`R`
~~~~~~~~~~~~~~~~~~~~

Motor resistance is usually small value, that you will not get accurately approximated with measuring tool like
multimeter. The best way to estimate it is to measure the current draw of the motor when applying a known voltage command, while keeping the motor blocked. Then, you can use Ohm's law to estimate the resistance :math:`R = \frac{V}{I}`.

Apparent inertia :math:`I`
~~~~~~~~~~~~~~~~~~~~~~~~~~

Apparent inertia is the rotor inertia of the motor, which is multiplied by the square of the gear ratio. While the reduction ratio is often known, it is not easy to guess the value of the rotor inertia.

This value can typically be adjusted across multiple BAM fits, monitor if the value saturates either up or low during
identification and ajust the range accordingly.

