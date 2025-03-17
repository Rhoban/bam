import numpy as np
from bam.actuator import Actuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum

class MXActuator(Actuator):
    """
    Represents a Dynamixel MX-64 or MX-106 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)

        # Input voltage and (firmware) gain
        self.vin: float = 15.0
        self.kp: float = 32.0

        # This gain, if multiplied by a position error and firmware KP, gives duty cycle
        # It was determined using an oscilloscope and MX actuators
        self.error_gain = 0.158

        # Maximum allowable duty cycle, also determined with oscilloscope
        self.max_pwm = 0.9625

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def control_unit(self) -> str:
        return "volts"

    def compute_control(
        self, position_error: float, q: float, dq: float
    ) -> float | None:
        duty_cycle = position_error * self.kp * self.error_gain
        duty_cycle = np.clip(duty_cycle, -self.max_pwm, self.max_pwm)

        return self.vin * duty_cycle

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        volts = control

        # Torque produced
        torque = self.model.kt.value * volts / self.model.R.value

        # Back EMF
        torque -= (self.model.kt.value**2) * dq / self.model.R.value

        return torque * torque_enable