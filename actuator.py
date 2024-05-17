from parameter import Parameter
import numpy as np
import message


class Actuator:
    def set_model(self, model):
        self.model = model
        self.initialize()

    def reset(self):
        pass

    def load_log(self, log: dict):
        """
        Called when a log is loaded, can be used to retrieve specific values (eg: input voltage, kp gain, etc.))
        """
        pass

    def initialize(self):
        raise NotImplementedError

    def control_unit(self) -> str:
        """
        Return the unit of the control signal (eg: "volts", "amps")
        """
        raise NotImplementedError

    def compute_control(self, position_error: float, q: float, dq: float) -> float | None:
        raise NotImplementedError

    def compute_torque(self, control: float | None, q: float, dq: float) -> float:
        raise NotImplementedError

    def compute_gravity_torque(self, q: float, mass: float, length: float) -> float:
        g = -9.80665
        return mass * g * length * np.sin(q)
    
    def get_extra_inertia(self) -> float:
        return 0.0

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

        # Maximum allowable PWM
        self.max_pwm = 0.9625

        # This gain, if multiplied by a position error and firmware KP, gives duty cycle
        self.error_gain = 0.158

    def load_log(self, log: dict):
        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):

        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 3.5)

        # Motor armature [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def control_unit(self) -> str:
        return "volts"

    def compute_control(self, position_error: float, q: float, dq: float) -> float | None:
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
                message.yellow("# WARNING: frictionloss should be updated dynamically with this model")
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
                print(message.yellow("# gearbox_torque is the torque applied to the gearbox"))
            if self.model.stribeck:
                print(message.yellow("# velocity is the angular velocity of the motor"))


actuators = {"mx": lambda: MXActuator()}
