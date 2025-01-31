import numpy as np
from .parameter import Parameter
from .testbench import Testbench, Pendulum
from . import message


class Actuator:
    def __init__(self, testbench_class: Testbench):
        self.testbench_class = testbench_class
        self.testbench: Testbench | None = None

    def set_model(self, model):
        self.model = model
        self.initialize()

    def reset(self):
        pass

    def load_log(self, log: dict):
        """
        Called when a log is loaded, can be used to retrieve specific values (eg: input voltage, kp gain, etc.))
        """
        self.testbench = self.testbench_class(log)

    def initialize(self):
        raise NotImplementedError

    def control_unit(self) -> str:
        """
        Return the unit of the control signal (eg: "volts", "amps")
        """
        raise NotImplementedError

    def compute_control(
        self, position_error: float, q: float, dq: float
    ) -> float | None:
        """
        The control (e.g volts or amps) produced by the actuator, given the position error and current configruation
        """
        raise NotImplementedError

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        """
        The torque [Nm] produced by the actuator, given the control signal and current configuration
        """
        raise NotImplementedError

    def get_extra_inertia(self) -> float:
        """
        Get apparent inertia
        """
        raise NotImplementedError

    def to_mujoco(self):
        raise NotImplementedError


class Erob(Actuator):
    def __init__(self, testbench_class: Testbench, damping=2.0):
        super().__init__(testbench_class)

        # Maximum current [A]
        self.max_amps = 12.0

        # Maximum input voltage [V]
        self.max_volts = 48.0

        # Damping factor
        self.damping = damping

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 15.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.1, 3.5)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 2.0)

        # Adjusting upper bounds for identification
        self.model.max_friction_base = 10.0
        self.model.max_load_friction = 1.0
        self.model.max_viscous_friction = 30.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "damping" in log:
            self.damping = log["damping"]

    def control_unit(self) -> str:
        return "amps"

    def compute_control(
        self, position_error: float, q: float, dq: float
    ) -> float | None:
        # Target velocity is assumed to be 0
        amps = position_error * self.kp + self.damping * np.sqrt(self.kp) * (0.0 - dq)
        amps = np.clip(amps, -self.max_amps, self.max_amps)

        return amps

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        # Computing the torque given the control signal
        # With eRob, control=None actually meany amps=0, and not a disconnection of the motor
        amps = control if control is not None else 0.0
        torque = self.model.kt.value * amps

        # Computing the torque boundaries given the maximum voltage and the back EMF
        volts_bounded_torque = (
            self.model.kt.value / self.model.R.value
        ) * self.max_volts
        emf = (self.model.kt.value**2) * dq / self.model.R.value

        min_torque = -volts_bounded_torque - emf
        max_torque = volts_bounded_torque - emf
        torque = np.clip(torque, min_torque, max_torque)

        return torque

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


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

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        # Volts to None means that the motor is disconnected
        if control is None:
            return 0.0

        volts = control

        # Torque produced
        torque = self.model.kt.value * volts / self.model.R.value

        # Back EMF
        torque -= (self.model.kt.value**2) * dq / self.model.R.value

        return torque


class XC330M288T(MXActuator):
    """
    XC330M288T
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)

        # Input voltage and (firmware) gain
        self.vin: float = 5.1
        self.kp: float = 1100.0

        # This gain, if multiplied by a position error and firmware KP, gives duty cycle
        # It was determined using an oscilloscope and XC actuators
        self.error_gain = 0.002877778

        # Maximum allowable duty cycle, also determined with oscilloscope
        self.max_pwm = 1.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 0.1, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 8.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.001, 0.0001, 0.01)

        # self.model.max_friction_base = 10.0
        # self.model.max_load_friction = 1.0
        # self.model.max_viscous_friction = 30.0


class STS3215(MXActuator):
    """
    Feetech STS3215 7.4v
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)

        # Input voltage and (firmware) gain
        self.vin: float = 7.4
        self.kp: float = 32

        # This gain, if multiplied by a position error and firmware KP, gives duty cycle
        # It was determined using an oscilloscope and STS3215 actuators
        # here, firmware_kp = kp
        # self.error_gain = 0.166
        self.error_gain = 0.001 * np.rad2deg(1.0)

        # Maximum allowable duty cycle, also determined with oscilloscope
        self.max_pwm = 1.0  # TODO, but can we assume 1.0 ?

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.784532, 0.05, 2.5)  # docs says 8 kg.cm / A
        # self.model.kt = Parameter(0.784532, 0.784531, 0.784533)  # docs says 8 kg.cm / A

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.1, 10.0)
        # self.model.R = Parameter(3, 2.9, 3.1)
        # self.model.R = 2.5 # docs says 2.5 ohm, Greg says 1.5

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0001, 0.00001, 0.04)

        # self.model.max_friction_base = 10.0
        # self.model.max_load_friction = 1.0
        # self.model.max_viscous_friction = 30.0

    def compute_control(
        self, position_error: float, q: float, dq: float
    ) -> float | None:
        duty_cycle = position_error * self.kp * self.error_gain
        duty_cycle = np.clip(duty_cycle, -self.max_pwm, self.max_pwm)

        return self.vin * duty_cycle


class LinearActuator(Actuator):
    """
    Represents a linear actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)

        # Input voltage and (firmware) gain
        self.vin: float = 12.0
        self.kp: float = 15.0

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
        duty_cycle = position_error * self.kp
        duty_cycle = np.clip(duty_cycle, -1.0, 1.0)

        return self.vin * duty_cycle

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        # Volts to None means that the motor is disconnected
        if control is None:
            return 0.0

        volts = control

        # Torque produced
        torque = self.model.kt.value * volts / self.model.R.value

        # Back EMF
        torque -= (self.model.kt.value**2) * dq / self.model.R.value

        return torque


actuators = {
    "mx64": lambda: MXActuator(Pendulum),
    "mx106": lambda: MXActuator(Pendulum),
    "erob80_100": lambda: Erob(Pendulum, damping=2.0),
    "erob80_50": lambda: Erob(Pendulum, damping=1.0),
    "xc330m288t": lambda: XC330M288T(Pendulum),
    "sts3215": lambda: STS3215(Pendulum),
}
