import numpy as np
import json
import dynamixel


class Parameter:
    def __init__(self, value: float, min: float, max: float, optimize: bool = True):
        # Current value of the parameter
        self.value: float = value

        # Minimum and maximum values for the parameter
        self.min: float = min
        self.max: float = max

        # Should this parameter be optimized?
        self.optimize: bool = optimize


class BaseModel:
    def compute_motor_torque(self, volts: float | None, dtheta: float) -> float:
        """
        This computes the torque applied by the motor, the friction torque and returns
        an expected angular acceleration.
        """
        raise NotImplementedError

    def reset(self) -> None:
        """
        Resets the model internal state
        """
        pass

    def compute_frictions(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> tuple:
        """
        This computes the friction torque applied by the system.
        Returns a tuple (frictionloss, damping)
        """
        raise NotImplementedError

    def get_extra_inertia(self) -> float:
        """
        This returns the extra inertia of the system.
        """
        return 0.0

    def get_parameters(self) -> list:
        """
        This returns the list of parameters that can be optimized.
        """
        return {
            name: param
            for name, param in vars(self).items()
            if isinstance(param, Parameter)
        }

    def get_parameter_values(self) -> dict:
        """
        Return a dict containing parameter values
        """
        parameters = self.get_parameters()
        x = {}
        for name in parameters:
            parameter = parameters[name]
            if parameter.optimize:
                x[name] = parameter.value
        return x

    def load_parameters(self, json_file: str) -> list:
        """
        Load parameters from a given filename
        """
        with open(json_file) as f:
            data = json.load(f)
            parameters = self.get_parameters()

            for name in parameters:
                if name in data:
                    parameters[name].value = data[name]


class Model(BaseModel):
    def __init__(
        self,
        load_dependent: bool = False,
        stribeck: bool = False,
        name: str = None,
    ):
        self.name = name

        # Model parameters
        self.load_dependent: bool = load_dependent
        self.stribeck: bool = stribeck

        # Torque constant [Nm/A] or [V/(rad/s)]
        self.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.R = Parameter(2.0, 1.0, 3.5)

        # Motor armature [kg m^2]
        self.armature = Parameter(0.005, 0.001, 0.05)

        # Base friction is always here, stribeck friction is added when not moving [Nm]
        self.friction_base = Parameter(0.05, 0.0, 0.2)
        if self.stribeck:
            self.friction_stribeck = Parameter(0.05, 0.0, 0.2)

        # Load-dependent friction, again base is always here and stribeck is added when not moving [Nm]
        if self.load_dependent:
            self.load_friction_base = Parameter(0.05, 0.0, 0.2)

            if self.stribeck:
                self.load_friction_stribeck = Parameter(0.05, 0.0, 1.0)

        if self.stribeck:
            # Stribeck velocity [rad/s] and curvature
            self.dtheta_stribeck = Parameter(0.2, 0.01, 3.0)
            self.alpha = Parameter(1.35, 0.5, 2.0)

        # Viscous friction [Nm/(rad/s)]
        self.friction_viscous = Parameter(0.1, 0.0, 1.0)

    def compute_motor_torque(self, volts: float | None, dtheta: float) -> float:
        # Volts to None means that the motor is disconnected
        if volts is None:
            return 0.0

        # Torque produced
        torque = self.kt.value * volts / self.R.value

        # Back EMF
        torque -= (self.kt.value**2) * dtheta / self.R.value

        return torque

    def compute_frictions(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> tuple:
        # Torque applied to the gearbox
        gearbox_torque = np.abs(external_torque - motor_torque)

        if self.stribeck:
            # Stribeck coeff (1 when stopped to 0 when moving)
            stribeck_coeff = np.exp(
                -(np.abs(dtheta / self.dtheta_stribeck.value) ** self.alpha.value)
            )

        # Static friction
        frictionloss = self.friction_base.value
        if self.load_dependent:
            frictionloss += self.load_friction_base.value * gearbox_torque

        if self.stribeck:
            frictionloss += stribeck_coeff * self.friction_stribeck.value

            if self.load_dependent:
                frictionloss += (
                    self.load_friction_stribeck.value * gearbox_torque * stribeck_coeff
                )

        # Viscous friction
        damping = self.friction_viscous.value

        return frictionloss, damping

    def get_extra_inertia(self) -> float:
        return self.armature.value

    def to_mujoco(self, volts: float, dynamixel_kp: float) -> None:
        # Armature
        armature = self.armature.value
        print(f" - Armature: {armature}")

        # Computing forcerange
        max_force = (self.kt.value / self.R.value) * volts
        print(f" - Forcerange: {max_force}")

        # Computing kp
        kp = max_force * dynamixel.ERROR_GAIN * dynamixel_kp
        print(f" - Kp: {kp}")

        # Computing damping
        damping = self.friction_viscous.value
        damping += self.R.value / (self.kt.value**2)
        print(f" - Damping: {damping}")

        # Computing frictionloss
        frictionloss = self.friction_base.value
        print(f" - Frictionloss: {frictionloss}")

        xml = f'<position kp="{kp}" forcerange="-{max_force} {max_force}" />\n'
        xml += f'<joint damping="{damping}" armature="{armature}" frictionloss="{frictionloss}" />'
        print("\nXML:")
        print(xml)

        if self.stribeck or self.load_dependent:
            print()
            print(
                "# WARNING: frictionloss should be updated dynamically with this model"
            )
            print("# Please use the following computation: ")
            if self.stribeck:
                print(
                    f"stribeck_coeff = exp(-(abs(velocity / {self.dtheta_stribeck.value}) ** {self.alpha.value}))"
                )

            print(f"frictionloss = {self.friction_base.value}")
            if self.load_dependent:
                print(
                    f"frictionloss += {self.load_friction_base.value} * gearbox_torque"
                )
            if self.stribeck:
                print(
                    f"frictionloss += stribeck_coeff * {self.friction_stribeck.value}"
                )
                if self.load_dependent:
                    print(
                        f"frictionloss += {self.load_friction_stribeck.value} * gearbox_torque * stribeck_coeff"
                    )

            print()
            if self.load_dependent:
                print("# gearbox_torque is the torque applied to the gearbox")
            if self.stribeck:
                print("# velocity is the angular velocity of the motor")


models = {
    "m1": lambda: Model(name="m1"),
    "m2": lambda: Model(name="m2", stribeck=True),
    "m3": lambda: Model(name="m3", load_dependent=True),
    "m4": lambda: Model(name="m4", load_dependent=True, stribeck=True),
}


def load_model(json_file: str):
    with open(json_file) as f:
        data = json.load(f)
        model = models[data["model"]]()
        model.load_parameters(json_file)
        return model
