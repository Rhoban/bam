from parameter import Parameter
import numpy as np


class Actuator:
    def set_model(self, model):
        self.model = model
        self.initialize()

    def reset(self):
        pass

    def load_log(self, log: dict):
        pass

    def initialize(self):
        raise NotImplementedError

    def control_unit(self) -> str:
        raise NotImplementedError

    def compute_control(
        self, position_error: float, dq: float, log_entry: dict, simulate_control: bool
    ) -> float | None:
        raise NotImplementedError

    def compute_torque(self, control: float | None, dq: float) -> float:
        raise NotImplementedError

    def to_mujoco(self):
        raise NotImplementedError


class MXActuator(Actuator):
    """
    Represents a Dynamixel MX-64 or MX-106 actuator
    """

    def __init__(self):
        # Input voltage
        self.vin: float = 15.0
        self.kp: float = 32.0

    def load_log(self, log: dict):
        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):

        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 3.5)

    def control_unit(self) -> str:
        return "volts"

    def compute_control(
        self, position_error: float, dq: float, log_entry: dict, simulate_control: bool
    ) -> float | None:
        # Maximum allowable PWM
        max_pwm = 0.9625
        # This gain, if multiplied by a position error and firmware KP, gives duty cycle
        error_gain = 0.158

        duty_cycle = position_error * self.kp * error_gain
        duty_cycle = np.clip(duty_cycle, -max_pwm, max_pwm)

        return self.vin * duty_cycle

    def compute_torque(self, control: float | None, dq: float) -> float:
        # Volts to None means that the motor is disconnected
        if control is None:
            return 0.0

        volts = control

        # Torque produced
        torque = self.model.kt.value * volts / self.model.R.value

        # Back EMF
        torque -= (self.model.kt.value**2) * dq / self.model.R.value

        return torque


actuators = {"mx": lambda: MXActuator()}
