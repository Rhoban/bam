# Copyright 2026 Theo Moore-Calters

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench


class ST3025Actuator(VoltageControlledActuator):
    """Waveshare ST3025 servo operated at 12 V."""

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=32,
            error_gain=0.166,
            max_pwm=0.97,
        )

    def initialize(self):
        self.model.kt = Parameter(0.784532, 0.05, 2.5)
        self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)
        self.model.R = Parameter(2.0, 0.1, 10.0)
        self.model.armature = Parameter(0.0001, 0.00001, 0.04)
        self.model.q_offset = Parameter(0, -0.2, 0.2)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

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
