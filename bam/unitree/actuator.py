import numpy as np
from typing import Union
from bam.message import yellow, print_parameter, bright
from bam.actuator import Actuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum


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
        self, q_target: float, q: float, dq: float, dt: float
    ) -> Union[float, None]:
        # Target velocity is assumed to be 0
        torque = (q_target - q) * self.kp * self.model.ratio.value + self.damping * (0.0 - dq)
        torque = np.clip(
            torque, -self.model.max_torque.value, self.model.max_torque.value
        )

        return torque

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        torques = control * torque_enable

        return torques

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def to_mujoco(self):
        if self.kp == 0:
            print(yellow(f"WARNING: kp is not set"))

        print_parameter("armature", self.model.armature.value)
        print_parameter("kp", self.kp * self.model.ratio.value)
        print_parameter("damping", self.model.friction_viscous.value)
        print_parameter("frictionloss", self.model.friction_base.value)
        print_parameter("forcerange", self.model.max_torque.value)

        print("")
