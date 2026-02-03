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

ENCODER_COUNTS_PER_REV = 4096
KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)
PWM_LIMIT = 885   # Default Present PWM limit for XL330


class XLActuator(VoltageControlledActuator):
    """
    Represents a Dynamixel xl330 or xc330 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            
            # Input voltage and (firmware) kP gain
            vin = 7.4,
            kp = 400,
            
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # Matches XL330 scaling: duty = error_gain * kp * error_rad
            # Using Kp divisor 256 (empirical) and default PWM limit 885
            error_gain = (ENCODER_COUNTS_PER_REV / (2 * np.pi)) / (KP_DIVISOR * PWM_LIMIT),
            # Maximum allowable duty cycle, also determined with oscilloscope   
            max_pwm = 1.0 # Seems to be this
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 0.3, 3.0) # TODO

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.5, 5.0) # TODO

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.0001, 0.05) # TODO

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
