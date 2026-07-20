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
            vin=15.0,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and MX actuators
            error_gain=0.158,
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=0.9625,
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
            vin=7.5,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and XL-320 actuators
            error_gain=0.05048199,
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=1.0,
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
XL330_PWM_LIMIT = 885  # Default Present PWM limit for XL330


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
            max_pwm=1.0
        )

    def initialize(self):
        self.model.kt = Parameter(1.6, 0.1, 3.0)
        self.model.R = Parameter(2.6, 2.0, 5.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


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

    def compute_torque(self, control, torque_enable, q, dq):
        """Torque from the regulated **bus** current (XL330-specific).

        Unlike the standard current-controlled servo (which senses and regulates
        the motor *phase* current, so ``torque = kt * current``), the XL330
        measures current on the *input/bus* side of the H-bridge. Its current
        loop therefore regulates the bus current, whose magnitude relates to the
        phase current through the PWM duty by power balance:

        .. math::

            I_\\text{bus} = \\text{duty} \\cdot I_\\text{phase},
            \\qquad I_\\text{phase} = \\frac{\\text{duty}\\,v_\\text{in} - k_t\\dot q}{R}

        Given the commanded bus current (``control``, signed by drive direction),
        we recover the duty that produces it — the positive root of
        ``vin*duty^2 - kt*dq*duty - R*|I_bus| = 0`` in the drive direction — clamp
        it to the PWM limit, and return ``torque = kt * I_phase``. This makes the
        delivered torque duty-dependent (``~sqrt(I_bus)`` at stall) rather than
        linear in the command, which is the behaviour observed in the data.

        .. warning::

            UNTESTED. The bus-current hypothesis is inferred from position-mode
            data (the ``|duty| ~ sqrt(|error|)`` relationship), not yet confirmed
            by a direct torque measurement, and this model has not been validated
            by a full identification against held-out xl330i data. The math is
            unit-tested for self-consistency and the numpy path runs, but the
            torch/mjlab path has not been executed on a real tensor backend.
            Treat the resulting parameters and simulations as provisional.
        """
        if control is None:
            return 0.0

        kt = self.model.kt.value
        R = self.model.R.value
        vin = self.vin

        # Solve the power-balance quadratic for the duty in the drive direction.
        # Uses the backend (numpy / torch) so it also runs under mjlab on tensors;
        # ``abs`` and ``**`` dispatch correctly for floats, ndarrays and tensors.
        sign = self.backend.sign(control)
        disc = (kt * dq) ** 2 + 4.0 * vin * R * abs(control)
        duty = (kt * dq + sign * disc**0.5) / (2.0 * vin)

        # PWM saturation (XL330 PWM Limit, full 885 range -> duty in [-1, 1]).
        duty = self.backend.clamp(duty, -1.0, 1.0)

        phase_current = (duty * vin - kt * dq) / R
        torque = kt * phase_current
        return torque * torque_enable

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
