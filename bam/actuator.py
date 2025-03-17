from .testbench import Testbench, Pendulum


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

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
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
        raise NotImplementedError("This actuator doesn't support to_mujoco")

from .erob.actuator import ErobActuator
from .dynamixel.actuator import MXActuator

actuators = {
    "mx64": lambda: MXActuator(Pendulum),
    "mx106": lambda: MXActuator(Pendulum),
    "erob80_100": lambda: ErobActuator(Pendulum, damping=2.0),
    "erob80_50": lambda: ErobActuator(Pendulum, damping=1.0),
}
