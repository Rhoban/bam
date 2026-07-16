# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Export an identified model to MuJoCo's built-in position actuator.

.. deprecated::
    This export is kept for convenience, but it is only an approximation: the
    identified model is collapsed into the constant ``kp``, ``damping``,
    ``frictionloss``, ``armature`` and ``forcerange`` of a MuJoCo position
    actuator, which cannot reproduce the load-dependent friction or the
    torque-dependent damping that BAM identifies. Prefer
    :class:`bam.mujoco.MujocoController` (CPU) or
    :class:`bam.mjlab.BamActuatorCfg` (GPU), which evaluate the actual model at
    each simulation step.
"""

import argparse

from .actuator import (
    Actuator,
    CurrentControlledActuator,
    VoltageControlledActuator,
)
from .message import bright, print_parameter, yellow
from .model import load_model
from .unitree.actuator import UnitreeGo1Actuator

DEPRECATION_NOTICE = """WARNING: to_mujoco is deprecated.

It approximates the model with a MuJoCo position actuator, and cannot reproduce
the load-dependent effects BAM identifies. Prefer bam.mujoco.MujocoController
(CPU) or bam.mjlab.BamActuatorCfg (GPU), which evaluate the model at each step.
"""


def voltage_controlled_to_mujoco(actuator: VoltageControlledActuator) -> dict:
    """Compute MuJoCo parameters for a voltage-controlled actuator.

    The back-EMF is folded into ``damping`` and the firmware P-gain into ``kp``,
    so the joint only matches the identified actuator to first order.
    """
    if actuator.vin == 0 or actuator.kp == 0:
        print(yellow("WARNING: kp or vin are not set"))

    kt = actuator.model.kt.value
    R = actuator.model.R.value

    return {
        "forcerange": actuator.vin * kt / R,
        "armature": actuator.model.armature.value,
        "kp": actuator.error_gain
        * actuator.kp
        * actuator.vin
        * actuator.max_pwm
        * kt
        / R,
        "damping": actuator.model.friction_viscous.value + kt**2 / R,
        "frictionloss": actuator.model.friction_base.value,
    }


def current_controlled_to_mujoco(actuator: CurrentControlledActuator) -> dict:
    """Compute MuJoCo parameters for a current-controlled actuator.

    ``forcerange`` accounts for both the voltage and the current limit, and the
    torque-dependent viscous damping is approximated by a constant ``damping``.
    """
    if actuator.vin == 0 or actuator.kp == 0:
        print(yellow("WARNING: kp or vin are not set"))

    kt = actuator.model.kt.value

    forcerange = actuator.vin * kt / actuator.model.R.value
    forcerange = min(forcerange, actuator.model.current_limit.value * kt)

    return {
        "forcerange": forcerange,
        "armature": actuator.model.armature.value,
        "kp": actuator.error_gain * actuator.kp * kt,
        "damping": (
            actuator.model.friction_viscous.value
            + actuator.model.viscous_damping_with_torque.value
        ),
        "frictionloss": actuator.model.friction_base.value,
    }


def unitree_go1_to_mujoco(actuator: UnitreeGo1Actuator) -> dict:
    """Compute MuJoCo parameters for a Unitree Go1 actuator."""
    if actuator.kp == 0:
        print(yellow("WARNING: kp is not set"))

    return {
        "forcerange": actuator.model.max_torque.value,
        "armature": actuator.model.armature.value,
        "kp": actuator.kp * actuator.model.ratio.value,
        "damping": actuator.model.friction_viscous.value,
        "frictionloss": actuator.model.friction_base.value,
    }


def to_mujoco(actuator: Actuator) -> dict:
    """Compute the MuJoCo position actuator parameters for ``actuator``.

    :param actuator: An identified actuator, with its log already loaded.
    :returns: A dict of MuJoCo attribute names to values.
    :raises NotImplementedError: If the actuator has no MuJoCo export.
    """
    if isinstance(actuator, CurrentControlledActuator):
        return current_controlled_to_mujoco(actuator)
    elif isinstance(actuator, VoltageControlledActuator):
        return voltage_controlled_to_mujoco(actuator)
    elif isinstance(actuator, UnitreeGo1Actuator):
        return unitree_go1_to_mujoco(actuator)

    raise NotImplementedError(f"{type(actuator).__name__} doesn't support to_mujoco")


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--params", type=str, default="params.json")
    arg_parser.add_argument("--kp", type=float, default=0.0)
    arg_parser.add_argument("--kd", type=float, default=0.0)
    arg_parser.add_argument("--vin", type=float, default=0.0)
    args = arg_parser.parse_args()

    print(yellow(DEPRECATION_NOTICE))

    # Load the model
    model = load_model(args.params)

    values = {
        "kp": args.kp,
        "kd": args.kd,
        "vin": args.vin,
        "mass": 0.0,
        "arm_mass": 0.0,
        "length": 0.0,
    }
    model.actuator.load_log(values)

    if model.name != "m1":
        print(
            yellow(
                f"WARNING: Model other than m1 can't be exported exactly to MuJoCo (model is {model.name})"
            )
        )

    bright(f"Reading model of type {model.name} from {args.params}")
    bright(f"Parameters export for MuJoCo, actuator {model.actuator_name}")

    for name, value in to_mujoco(model.actuator).items():
        print_parameter(name, value)

    print("")
