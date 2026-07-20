# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
import json
from pathlib import Path
from .actuator import Actuator
from .actuators import actuators
from .parameter import Parameter


class Model:
    """Friction model for a servo actuator.

    Combines a motor model (set via :meth:`set_actuator`) with a friction budget
    that maps joint state and torques to a maximum resistive torque
    :math:`\\tau_{fm}`. The friction applied by the simulator is the stopping
    torque clipped in :math:`[-\\tau_{fm}, \\tau_{fm}]`.

    Use :func:`load_model` to instantiate from a parameter file rather than
    constructing directly.

    :param load_dependent: Enable load-dependent friction terms.
    :param directional: Enable directional (motor-side vs external-side) load friction.
        Requires ``load_dependent=True``.
    :param stribeck: Enable Stribeck effect (higher friction near zero velocity).
    :param quadratic: Enable quadratic load-friction coupling term.
        Requires ``directional=True`` and ``stribeck=True``.
    :param name: Model variant identifier, e.g. ``"m6"``.
    :param title: Human-readable title used in plots.
    """

    def __init__(
        self,
        load_dependent: bool = False,
        directional: bool = False,
        stribeck: bool = False,
        quadratic: bool = False,
        name: str = None,
        title: str = "",
    ):
        self.actuator_name = None
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
        """Attach an actuator to this model and initialize its parameters.

        :param actuator: Actuator instance to attach. Determines the motor
            model (kt, R, kp, vin) and the set of friction parameters that
            are created on this model.
        """
        self.actuator = actuator
        self.actuator.set_model(self)

        # Offset of the motor (testbench error)
        self.q_offset = Parameter(0.0, -0.1, 0.1)

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
            self.dtheta_stribeck = Parameter(0.2, 0.10, 1.0)
            self.alpha = Parameter(1.35, 1.0, 4.0)

        # Viscous friction [Nm/(rad/s)]
        self.friction_viscous = Parameter(0.1, 0.0, self.max_viscous_friction)

    def compute_frictions(
        self, motor_torque: float, external_torque: float, dtheta: float
    ) -> tuple:
        """Compute the friction budget for the current state.

        Returns the two MuJoCo friction parameters that implement the BAM
        friction-budget formulation: ``frictionloss`` (Coulomb-like constant
        term) and ``damping`` (viscous term).  The simulator applies friction
        by clipping the stopping torque in
        :math:`[-\\text{frictionloss} - \\text{damping}\\cdot|\\dot{\\theta}|,\\ +\\ldots]`.

        :param motor_torque: Torque produced by the motor [Nm].
        :param external_torque: External (gravity/load) torque seen at the joint [Nm].
        :param dtheta: Joint velocity [rad/s].
        :returns: Tuple ``(frictionloss, damping)`` ready to be written into
            ``mj_model.dof_frictionloss`` and ``mj_model.dof_damping``.
        """
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

                if self.quadratic:
                    enable_quadratic = np.sign(external_torque) != np.sign(motor_torque)
                    direction_motor = np.abs(external_torque) < np.abs(motor_torque)
                    direction_external = np.abs(external_torque) > np.abs(motor_torque)

                    gearbox_torque2_motor = (
                        self.load_friction_external_quad.value
                        * np.abs(external_torque) ** 2
                    )
                    gearbox_torque2_external = (
                        self.load_friction_motor_quad.value * np.abs(motor_torque) ** 2
                    )

                    frictionloss += (
                        stribeck_coeff
                        * (
                            direction_motor * gearbox_torque2_motor
                            + direction_external * gearbox_torque2_external
                        )
                        * enable_quadratic
                    )

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
            self.load_parameters_from_dict(data)

    def load_parameters_from_dict(self, data: dict) -> list:
        """
        Load parameters from a given dict
        """
        parameters = self.get_parameters()

        for name in parameters:
            if name in data:
                parameters[name].value = data[name]


class DummyModel(Model):
    def __init__(self):
        super().__init__()


models = {
    "m1": lambda: Model(name="m1", title="Coulomb (M1)"),
    "m2": lambda: Model(name="m2", stribeck=True, title="Stribeck (M2)"),
    "m3": lambda: Model(name="m3", load_dependent=True, title="Load-dependent (M3)"),
    "m4": lambda: Model(
        name="m4",
        load_dependent=True,
        stribeck=True,
        title="Stribeck load-dependent (M4)",
    ),
    "m5": lambda: Model(
        name="m5",
        load_dependent=True,
        stribeck=True,
        directional=True,
        title="Stribeck load-dependent directional (M5)",
    ),
    "m6": lambda: Model(
        name="m6",
        load_dependent=True,
        stribeck=True,
        directional=True,
        quadratic=True,
        title="Stribeck load-dependent directional quadratic (M6)",
    ),
}


def _resolve_json_path(
    json_file: str | None, motor_name: str | None, model: str | None
) -> str:
    if json_file is not None:
        return json_file
    if motor_name is None or model is None:
        raise ValueError("Provide either json_file or both motor_name and model.")
    params_root = Path(__file__).parent / "params"
    path = params_root / motor_name / f"{model}.json"
    if not path.exists():
        motor_dir = params_root / motor_name
        available_models = (
            sorted(p.stem for p in motor_dir.glob("*.json"))
            if motor_dir.exists()
            else []
        )
        available_motors = (
            sorted(d.name for d in params_root.iterdir() if d.is_dir())
            if params_root.exists()
            else []
        )
        raise FileNotFoundError(
            f"No bundled params for motor={motor_name!r} model={model!r}. "
            f"Available models for this motor: {available_models}. "
            f"Available motors: {available_motors}."
        )
    return str(path)


def load_model(
    json_file: str = None, *, motor_name: str = None, model: str = None
) -> Model:
    """Load a BAM friction model from a parameter file.

    Specify the source with **one** of two mutually exclusive approaches:

    - **Bundled motor** — pass ``motor_name`` and ``model``::

        model = load_model(motor_name="xl330", model="m6")

    - **Custom JSON** — pass ``json_file`` (output of ``bam.fit``)::

        model = load_model("params/my_motor/m6.json")

    :param json_file: Path to a BAM params JSON file.
    :param motor_name: Name of a bundled motor (e.g. ``"xl330"``, ``"mx106"``).
        Must be combined with ``model``.
    :param model: Model variant for a bundled motor (``"m1"``–``"m6"``).
        Must be combined with ``motor_name``.
    :returns: A :class:`Model` instance with all parameters loaded.
    :raises FileNotFoundError: If the requested bundled motor or model does not exist.
    """
    path = _resolve_json_path(json_file, motor_name, model)
    with open(path) as f:
        data = json.load(f)
        return load_model_from_dict(data)


def load_model_from_dict(data: dict) -> Model:
    """Load a BAM friction model from a parameter dictionary.

    This is the low-level counterpart of :func:`load_model`, used when the
    JSON data has already been parsed.

    :param data: Dictionary produced by ``json.load`` on a BAM params file.
        Must contain at least ``"model"`` and ``"actuator"`` keys.
    :returns: A :class:`Model` instance with all parameters loaded.
    """
    model = models[data["model"]]()
    model.set_actuator(actuators[data["actuator"]]())
    model.actuator_name = data["actuator"]
    model.load_parameters_from_dict(data)
    return model
