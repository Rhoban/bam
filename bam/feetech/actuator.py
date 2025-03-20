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
            vin=7.4,
            kp=32,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and STS3215 actuators
            # here, firmware_kp = kp
            error_gain=0.166,
            # self.error_gain = 0.001 * np.rad2deg(1.0)
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=0.97,  # TODO, but can we assume 1.0 ?
        )

        self.default_max_velocity = (3400 * 2 * np.pi) / 4096

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def load_log(self, log: dict):
        super().load_log(log)

        self.q_target_smooth = np.zeros_like(self.kp)

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.784532, 0.05, 2.5)  # docs says 8 kg.cm / A

        self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.1, 10.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0001, 0.00001, 0.04)

        self.model.q_offset = Parameter(0, -0.2, 0.2)

        self.model.max_velocity = Parameter(
            self.default_max_velocity,
            0.1 * self.default_max_velocity,
            10.0 * self.default_max_velocity,
        )

    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """
        Assumes the motor is using a kp controller
        This can be overloaded if more custom behaviour is used
        """
        
        # Internal target position is clipped using maximum velocity
        self.q_target_smooth = np.clip(
            q_target,
            self.q_target_smooth - self.model.max_velocity.value * dt,
            self.q_target_smooth + self.model.max_velocity.value * dt,
        )

        duty_cycle = (
            (self.q_target_smooth - q)
            * self.kp
            * self.error_gain
            * self.model.error_gain_ratio.value
        )
        duty_cycle = np.clip(duty_cycle, -self.max_pwm, self.max_pwm)

        return self.vin * duty_cycle
