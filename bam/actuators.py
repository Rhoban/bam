from .testbench import Pendulum
from .erob.actuator import ErobActuator
from .dynamixel.actuator import MXActuator
from .feetech.actuator import STS3215Actuator
from .unitree.actuator import UnitreeGo1Actuator

actuators = {
    # Dynamixel MX series
    "mx64": lambda: MXActuator(Pendulum),
    "mx106": lambda: MXActuator(Pendulum),
    
    # eRob actuators with custom PD controller (mostly used in experiments to model
    # frictions)
    "erob80_100": lambda: ErobActuator(Pendulum, damping=2.0),
    "erob80_50": lambda: ErobActuator(Pendulum, damping=1.0),

    # Feetech STS3215
    "sts3215": lambda: STS3215Actuator(Pendulum),

    # Unitree Go1
    "unitree_go1": lambda: UnitreeGo1Actuator(Pendulum)
}