# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from bam.message import yellow, print_parameter, bright
from bam.actuator import VoltageControlledActuator, CurrentControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum

class MXActuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-64 or MX-106 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin = 15.0,
            kp = 32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and MX actuators
            error_gain = 0.158,
            # Maximum allowable duty cycle, also determined with oscilloscope   
            max_pwm = 0.9625
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XL320Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XL-320 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin = 7.5,
            kp = 32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and XL-320 actuators
            error_gain = 0.05048199,
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm = 1.0
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.7, 0.25, 1.5)

        # Motor resistance [Ohm]
        self.model.R = Parameter(5.0, 3.0, 40.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0005, 0.0001, 0.01)

        self.model.max_load_friction = 1.0

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


XL330_ENCODER_COUNTS_PER_REV = 4096
XL330_KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)
XL330_PWM_LIMIT = 885   # Default Present PWM limit for XL330


class XL330Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XL330 actuator.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=400,
            error_gain=(XL330_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (XL330_KP_DIVISOR * XL330_PWM_LIMIT),
            max_pwm=1.0,
            # Firmware current limit [A] for the XL330
            max_current=1.75,
        )

    def initialize(self):
        self.model.kt = Parameter(1.6, 0.1, 3.0)
        self.model.R = Parameter(2.6, 2.0, 5.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XL330SupplyCurrentLimitedActuator(XL330Actuator):
    """
    Dynamixel XL330 in position control with a firmware limit on the **supply**
    current, as enforced by ROBOTIS's custom "impact guard" firmware V4 (the
    factory firmware only enforces Current Limit in the current-based modes).

    The XL330 senses current on the supply side of the H-bridge. By power balance
    the reported current is ``|duty| * I_phase``, so holding it under the limit
    bounds the duty cycle to the roots of

    .. math::

        v_\\text{in}\\,\\text{duty}^2 - k_t \\dot q\\,\\text{duty} - R\\,I_\\text{lim} \\le 0

    i.e. ``duty`` in ``[(k_t dq - S) / (2 v_in), (k_t dq + S) / (2 v_in)]`` with
    ``S = sqrt((k_t dq)^2 + 4 v_in R I_lim)``. Unlike a phase-current limit, the
    allowed duty grows with the square root of the limit and does not vanish
    at stall.

    ``current_limit`` is identified as an *effective* limit: ``kt`` and ``R`` are
    themselves effective values fitted on the mechanical response, so the limit
    that reproduces the firmware's duty clamp needs not be exactly the register
    value (1000 mA on the robot).
    """

    def initialize(self):
        super().initialize()
        self.model.current_limit = Parameter(1.0, 0.5, 3.0)

    def compute_control(self, q_target, q, dq, dt):
        duty_cycle = (q_target - q) * self.kp * self.error_gain

        kt = self.model.kt.value
        R = self.model.R.value
        back_emf = kt * dq
        span = (back_emf**2 + 4.0 * self.vin * R * self.model.current_limit.value) ** 0.5
        duty_cycle = self.backend.clamp(
            duty_cycle,
            (back_emf - span) / (2.0 * self.vin),
            (back_emf + span) / (2.0 * self.vin),
        )

        duty_cycle = self.backend.clamp(duty_cycle, -self.max_pwm, self.max_pwm)
        self.duty_cycle = duty_cycle

        return self.vin * duty_cycle


class XL330CurrentActuator(CurrentControlledActuator):
    """
    Represents a Dynamixel XL330 actuator controlled in current position mode.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=400,
            error_gain=0.01,
        )

    def initialize(self):
        self.model.kt = Parameter(1.6, 0.1, 3.0)
        self.model.R = Parameter(2.6, 2.0, 5.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)
        self.model.current_limit = Parameter(1.5, 1.0, 3.0)
        self.model.viscous_damping_with_torque = Parameter(0.0, 0.0, 0.1)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
