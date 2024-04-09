import numpy as np
import json


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

    def compute_friction_torque(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> float:
        """
        This computes the friction torque applied by the system.
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
    def __init__(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.kt = Parameter(1.6, 0.1, 10.0, optimize=False)

        # Motor resistance [Ohm]
        self.R = Parameter(2.0, 0.1, 10.0, optimize=False)

        # Motor armature [kg m^2]
        self.armature = Parameter(0.005, 0.001, 0.05)

        # Base friction is always here, stribeck friction is added when not moving [Nm]
        self.friction_base = Parameter(0.05, 0.005, 0.5)
        self.friction_stribeck = Parameter(0.05, 0.005, 0.5)

        # Load-dependent friction, again base is always here and stribeck is added when not moving [Nm]
        self.load_friction_base = Parameter(0.05, 0.005, 1.0)
        self.load_friction_stribeck = Parameter(0.05, 0.005, 1.0)

        # Stribeck velocity [rad/s] and curvature
        self.dtheta_stribeck = Parameter(0.2, 0.01, 10.0)
        self.alpha = Parameter(1.35, 0.5, 2.0)

        # Viscous friction [Nm/(rad/s)]
        self.friction_viscous = Parameter(0.1, 0.0, 2.0)

    def compute_motor_torque(self, volts: float | None, dtheta: float) -> float:
        # Volts to None means that the motor is disconnected
        if volts is None:
            return 0.0

        # Torque produced
        torque = self.kt.value * volts / self.R.value

        # Back EMF
        torque -= (self.kt.value**2) * dtheta / self.R.value

        return torque

    def compute_friction_torque(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> float:

        # Torque applied to the gearbox
        gearbox_torque = np.abs(external_torque - motor_torque) 

        # Stribeck coeff (1 when stopped to 0 when moving)
        stribeck_coeff = np.exp(
            -(np.abs(dtheta / self.dtheta_stribeck.value) ** self.alpha.value)
        )

        # Static friction
        static_friction = (
            self.friction_base.value + self.load_friction_base.value * gearbox_torque
        )
        static_friction += stribeck_coeff * (
            self.friction_stribeck.value
            + self.load_friction_stribeck.value * gearbox_torque
        )
        net_torque = motor_torque + external_torque

        # Adjust the static friction to compensate for the net torque
        if net_torque > 0:
            static_friction = -min(static_friction, net_torque)
        else:
            static_friction = -max(-static_friction, net_torque)

        # Viscous friction
        damping_friction = -self.friction_viscous.value * dtheta

        return damping_friction + static_friction

    def get_extra_inertia(self) -> float:
        return self.armature.value
