import numpy as np
import json
from actuator import Actuator, actuators
from parameter import Parameter


class Model:
    def __init__(
        self,
        load_dependent: bool = False,
        directional: bool = False,
        stribeck: bool = False,
        quadratic: bool = False,
        name: str = None,
        title: str = "",
    ):
        self.name = name
        self.title = title

        # Model parameters
        self.load_dependent: bool = load_dependent
        self.directional: bool = directional
        self.stribeck: bool = stribeck
        self.quadratic: bool = quadratic

        self.max_friction_base = 0.2
        self.max_load_friction = 0.5
        self.max_viscous_friction = 1.0

    def reset(self) -> None:
        """
        Resets the model internal state
        """
        self.actuator.reset()

    def set_actuator(self, actuator: Actuator) -> None:
        self.actuator = actuator
        self.actuator.set_model(self)

        # Base friction is always here, stribeck friction is added when not moving [Nm]
        self.friction_base = Parameter(0.05, 0.0, self.max_friction_base)
        if self.stribeck:
            self.friction_stribeck = Parameter(0.05, 0.0, self.max_friction_base)

        # Load-dependent friction, again base is always here and stribeck is added when not moving [Nm]
        if self.load_dependent:
            if self.directional:
                self.load_friction_motor = Parameter(0.05, 0.0, self.max_load_friction)
                self.load_friction_external = Parameter(
                    0.05, 0.0, self.max_load_friction
                )
            else:
                self.load_friction_base = Parameter(0.05, 0.0, self.max_load_friction)

            if self.stribeck:
                if self.directional:
                    self.load_friction_motor_stribeck = Parameter(0.05, 0.0, 1.0)
                    self.load_friction_external_stribeck = Parameter(0.05, 0.0, 1.0)
                else:
                    self.load_friction_stribeck = Parameter(0.05, 0.0, 1.0)

                if self.quadratic:
                    self.load_friction_motor_quad = Parameter(0.0, 0.0, 0.01)
                    self.load_friction_external_quad = Parameter(0.0, 0.0, 0.01)

        if self.stribeck:
            # Stribeck velocity [rad/s] and curvature
            self.dtheta_stribeck = Parameter(0.2, 0.01, 3.0)
            self.alpha = Parameter(1.35, 0.5, 3.0)

        # Viscous friction [Nm/(rad/s)]
        self.friction_viscous = Parameter(0.1, 0.0, self.max_viscous_friction)

    def compute_frictions(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> tuple:
        # Torque applied to the gearbox
        if self.directional:
            gearbox_torque = np.abs(
                external_torque * self.load_friction_external.value
                - motor_torque * self.load_friction_motor.value
            )
            if self.stribeck:
                gearbox_torque_stribeck = np.abs(
                    external_torque * self.load_friction_external_stribeck.value
                    - motor_torque * self.load_friction_motor_stribeck.value
                )
        else:
            gearbox_torque = np.abs(external_torque - motor_torque)

        if self.stribeck:
            # Stribeck coeff (1 when stopped to 0 when moving)
            stribeck_coeff = np.exp(
                -(np.abs(dtheta / self.dtheta_stribeck.value) ** self.alpha.value)
            )

        # Static friction
        frictionloss = self.friction_base.value
        if self.load_dependent:
            if self.directional:
                frictionloss += gearbox_torque
            else:
                frictionloss += self.load_friction_base.value * gearbox_torque

        if self.stribeck:
            frictionloss += stribeck_coeff * self.friction_stribeck.value

            if self.load_dependent:
                if self.directional:
                    frictionloss += gearbox_torque_stribeck * stribeck_coeff
                else:
                    frictionloss += (
                        self.load_friction_stribeck.value
                        * gearbox_torque
                        * stribeck_coeff
                    )

                if self.quadratic and np.sign(external_torque) != np.sign(motor_torque):
                    if abs(external_torque) < abs(motor_torque):
                        gearbox_torque2 = (
                            self.load_friction_external_quad.value
                            * abs(external_torque) ** 2
                        )
                    else:
                        gearbox_torque2 = (
                            self.load_friction_motor_quad.value * abs(motor_torque) ** 2
                        )

                    frictionloss += gearbox_torque2 * stribeck_coeff

        # Viscous friction
        damping = self.friction_viscous.value

        return frictionloss, damping

    def get_parameters(self) -> dict:
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


class DummyModel(Model):
    def __init__(self):
        super().__init__()


models = {
    "m1": lambda: Model(name="m1", title="Coulomb"),
    "m2": lambda: Model(name="m2", stribeck=True, title="Stribeck"),
    "m3": lambda: Model(name="m3", load_dependent=True, title="Load-dependent"),
    "m4": lambda: Model(
        name="m4", load_dependent=True, stribeck=True, title="Stribeck load-dependent"
    ),
    "m5": lambda: Model(
        name="m5",
        load_dependent=True,
        stribeck=True,
        directional=True,
        title="Stribeck load-dependent directional",
    ),
    "m6": lambda: Model(
        name="m6",
        load_dependent=True,
        stribeck=True,
        directional=True,
        quadratic=True,
        title="Stribeck load-dependent directional quadratic",
    ),
}


def load_model(json_file: str):
    with open(json_file) as f:
        data = json.load(f)
        model = models[data["model"]]()
        model.set_actuator(actuators[data["actuator"]]())
        model.load_parameters(json_file)
        return model
