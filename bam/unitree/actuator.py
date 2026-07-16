# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING, Union
from bam.actuator import Actuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum

if TYPE_CHECKING:
    from bam.actuator import ArrayLike


class UnitreeGo1Actuator(Actuator):
    def __init__(self, testbench_class: Testbench, damping=0.3):
        super().__init__(testbench_class)

        # Damping factor
        self.damping = damping

    def initialize(self):
        # Torque multiplier
        self.model.ratio = Parameter(1.0, 0.5, 2.0)

        # Maximum torque [N.m]
        self.model.max_torque = Parameter(23.7, 20, 30)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.00001, 2.0)

        # Adjusting upper bounds for identification
        self.model.max_friction_base = 5.0
        self.model.max_load_friction = 2.0
        self.model.max_viscous_friction = 1.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "damping" in log:
            self.damping = log["damping"]

    def control_unit(self) -> str:
        return "N.m"

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> Union[ArrayLike, None]:
        # Target velocity is assumed to be 0
        torque = (q_target - q) * self.kp * self.model.ratio.value + self.damping * (0.0 - dq)
        torque = self.backend.clamp(
            torque, -self.model.max_torque.value, self.model.max_torque.value
        )

        return torque

    def compute_torque(
        self, control: ArrayLike | None, torque_enable: bool, q: ArrayLike, dq: ArrayLike
    ) -> ArrayLike:
        torques = control * torque_enable

        return torques

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
