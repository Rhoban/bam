from parameter import Parameter
import numpy as np
from testbench import Testbench, Pendulum
import message


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
    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class)

        # Maximum current [A]
        self.max_amps = 12.0

        # Maximum input voltage [V]
        self.max_volts = 48.0

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 15.0)

        # Motor resistance [Ohm]
        # self.model.R = Parameter(2.0, 0.1, 3.5)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 2.0)

        # Adjusting upper bounds for identification
        self.model.max_friction_base = 10.0
        self.model.max_load_friction = 1.0
        self.model.max_viscous_friction = 30.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

    def control_unit(self) -> str:
        return "amps"

    def compute_control(
        self, position_error: float, q: float, dq: float
    ) -> float | None:
        # Target velocity is assumed to be 0
        amps = position_error * self.kp + 2 * np.sqrt(self.kp) * (0.0 - dq)
        amps = np.clip(amps, -self.max_amps, self.max_amps)

        return amps

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        # Computing the torque given the control signal
        # With eRob, control=None actually meany amps=0, and not a disconnection of the motor
        amps = control if control is not None else 0.0
        torque = self.model.kt.value * amps

        # Computing the torque boundaries given the maximum voltage and the back EMF
        # volts_bounded_torque = (
        #     self.model.kt.value / self.model.R.value
        # ) * self.max_volts
        # emf = (self.model.kt.value**2) * dq / self.model.R.value

        # min_torque = -volts_bounded_torque - emf
        # max_torque = volts_bounded_torque - emf
        # torque = np.clip(torque, min_torque, max_torque)

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
        self.model.R = Parameter(2.0, 1.0, 3.5)

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

    def to_mujoco(self) -> None:
        # TODO: This should be moved elsewhere, extra models (M5, M6) should be also handled
        raise NotImplementedError

        message.bright("MX Actuator")
        message.print_parameter("Input voltage", self.vin)
        message.print_parameter("Firmware KP", self.kp)

        message.bright("Parameters")
        # Armature
        armature = self.model.armature.value
        message.print_parameter("Armature", armature)

        # Computing forcerange
        max_force = (self.model.kt.value / self.model.R.value) * self.vin
        message.print_parameter("Force range", max_force)

        # Computing kp
        kp = max_force * self.error_gain * self.kp
        message.print_parameter("Kp", kp)

        # Computing damping
        damping = self.model.friction_viscous.value
        damping += self.model.R.value / (self.model.kt.value**2)
        message.print_parameter("Damping", damping)

        # Computing frictionloss
        frictionloss = self.model.friction_base.value
        message.print_parameter("Frictionloss", frictionloss)

        xml = f'<position kp="{kp}" forcerange="-{max_force} {max_force}" />\n'
        xml += f'<joint damping="{damping}" armature="{armature}" frictionloss="{frictionloss}" />'
        message.bright("XML:")
        print(message.emphasis(xml))

        if self.model.stribeck or self.model.load_dependent:
            message.bright("Runtime updates required:")
            print(
                message.yellow(
                    "# WARNING: frictionloss should be updated dynamically with this model"
                )
            )
            print(message.yellow("# Please use the following computation: "))
            if self.model.stribeck:
                print(
                    f"stribeck_coeff = exp(-(abs(velocity / {self.model.dtheta_stribeck.value}) ** {self.model.alpha.value}))"
                )

            print(f"frictionloss = {self.model.friction_base.value}")
            if self.model.load_dependent:
                print(
                    f"frictionloss += {self.model.load_friction_base.value} * gearbox_torque"
                )
            if self.model.stribeck:
                print(
                    f"frictionloss += stribeck_coeff * {self.model.friction_stribeck.value}"
                )
                if self.model.load_dependent:
                    print(
                        f"frictionloss += {self.model.load_friction_stribeck.value} * gearbox_torque * stribeck_coeff"
                    )

            print()
            if self.model.load_dependent:
                print(
                    message.yellow(
                        "# gearbox_torque is the torque applied to the gearbox"
                    )
                )
            if self.model.stribeck:
                print(message.yellow("# velocity is the angular velocity of the motor"))


actuators = {
    "mx64": lambda: MXActuator(Pendulum),
    "mx106": lambda: MXActuator(Pendulum),
    "erob": lambda: Erob(Pendulum),
}
