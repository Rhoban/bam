# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from bam.message import yellow, print_parameter, bright
from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum


class MXActuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-64 or MX-106 actuator.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=15.0,
            kp=32.0,
            # Determined using an oscilloscope on MX actuators
            error_gain=0.158,
            max_pwm=0.9625,
        )

    def initialize(self):
        self.model.kt      = Parameter(1.6, 1.0, 3.0)
        self.model.R       = Parameter(2.0, 1.0, 5.0)
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XL320Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XL-320 actuator.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=32.0,
            # Determined using an oscilloscope on XL-320 actuators
            error_gain=0.05048199,
            max_pwm=1.0,
        )

    def initialize(self):
        self.model.kt               = Parameter(0.25, 0.7, 1.5)
        self.model.R                = Parameter(5.0, 7.0, 40.0)
        self.model.armature         = Parameter(0.0005, 0.0001, 0.01)
        self.model.max_load_friction = 1.0

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XM430Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XM430-W350 actuator.

    Datasheet values at 12.0 V:
      - Stall torque:   4.1 Nm at 2.3 A  →  kt_init ≈ 1.78 Nm/A
      - R_eff estimate: 12 V / 2.3 A     ≈  5.2 Ω
      - No-load speed:  46 rpm

    error_gain = 1/128 is derived from the XM430 Position P Gain table
    conversion (Kp_actual = Kp_TBL / 128).  This is an initial approximation;
    calibrate with oscilloscope data if higher accuracy is needed.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=800.0,
            error_gain=1.0 / 128.0,
            max_pwm=1.0,
        )

    def initialize(self):
        # Output-shaft torque constant from datasheet ≈ 1.78 Nm/A
        self.model.kt       = Parameter(1.78, 1.0, 3.0)
        # Effective resistance: 12 V / 2.3 A ≈ 5.2 Ω; broad bounds for fitting
        self.model.R        = Parameter(5.2, 2.0, 10.0)
        # Reflected inertia: unknown, use broad bounds
        self.model.armature = Parameter(0.005, 0.0001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
