# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

from __future__ import annotations

from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench


class FeetechHLSActuator(VoltageControlledActuator):
    """Feetech HLS2915 in firmware Position mode.

    The HLS firmware converts a position error to PWM duty. BAM therefore models
    it as a voltage-controlled actuator: the fitted ``error_gain_ratio`` maps
    the firmware ``kp`` register into duty cycle, then the DC motor equation
    accounts for winding resistance and back-EMF.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=8.0,
            # HLS uses the same 0--255 gain register convention as SMS/STS
            # servos. The fitted ratio absorbs the vendor's undocumented
            # register-to-duty scaling.
            error_gain=0.166,
            max_pwm=1.0,
            max_current=1.37,
        )

    def initialize(self):
        # Datasheet stall and no-load specifications both put Kt near 0.97 Nm/A.
        self.model.kt = Parameter(0.97, 0.5, 1.6)

        # Position-mode PWM makes R observable; initialize from the two bench
        # measurements (7.06 and 7.56 ohm) and let the voltage data identify it.
        self.model.R = Parameter(7.5, 4.0, 11.0)

        # Convert the firmware Kp register to duty. The HLS manual does not
        # specify this scale, so it must be identified from position-mode logs.
        self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)

        self.model.armature = Parameter(0.001, 0.00001, 0.006)
        self.model.q_offset = Parameter(0.0, -0.5, 0.5)

        # Bounds scale with the ~0.5 Nm operating torque on this pendulum.
        self.model.max_friction_base = 0.08
        self.model.max_load_friction = 0.15
        self.model.max_viscous_friction = 0.2

    def compute_control(self, q_target, q, dq, dt):
        duty_cycle = (
            (q_target - q)
            * self.kp
            * self.error_gain
            * self.model.error_gain_ratio.value
        )
        duty_cycle = self.backend.clamp(duty_cycle, -self.max_pwm, self.max_pwm)
        self.duty_cycle = duty_cycle
        return self.vin * duty_cycle

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
