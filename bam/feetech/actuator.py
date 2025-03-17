import numpy as np
from typing import Union
from bam.message import yellow, print_parameter, bright
from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum

class STS3215Actuator(VoltageControlledActuator):
    """
    Feetech STS3215 7.4v
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin = 7.4,
            kp = 32,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and STS3215 actuators
            # here, firmware_kp = kp
            error_gain = 0.166,
            # self.error_gain = 0.001 * np.rad2deg(1.0)

            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm = 0.97  # TODO, but can we assume 1.0 ?
            
        )

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.784532, 0.05, 2.5)  # docs says 8 kg.cm / A

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.1, 10.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0001, 0.00001, 0.04)

        self.model.q_offset = Parameter(0, -0.2, 0.2)
