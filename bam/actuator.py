# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from .testbench import Testbench
from bam.parameter import Parameter
from .message import yellow, print_parameter


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
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """
        The control (e.g volts or amps) produced by the actuator, given the position error and current configruation
        """
        raise NotImplementedError

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
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
        raise NotImplementedError("This actuator doesn't support to_mujoco")


class VoltageControlledActuator(Actuator):
    def __init__(self, testbench_class: Testbench, vin: float, kp: float, error_gain: float, max_pwm: float = 1.0):
        super().__init__(testbench_class)

        # Input voltage
        self.vin = vin

        # Proportional gain
        self.kp = kp

        # Error gain is such that
        # duty_cycle = error_gain * kp * angular_error
        # results in the target duty_cycle (between 0 and 1)
        self.error_gain = error_gain

        # Maximum PWM allowed by the controller, typically close to 1.0
        self.max_pwm = max_pwm

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "vin" in log:
            self.vin = log["vin"]

    def initialize(self):
        """
        Default initialization for kt and R that should be set in a voltage controlled servo
        """
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.0, 0.0, 10.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(1.0, 0.0, 10.0)

    def control_unit(self) -> str:
        return "volts"
    
    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        """
        Assumes the motor is using a kp controller
        This can be overloaded if more custom behaviour is used
        """
        duty_cycle = (q_target - q) * self.kp * self.error_gain
        duty_cycle = np.clip(duty_cycle, -self.max_pwm, self.max_pwm)

        return self.vin * duty_cycle
    
    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        """
        Computes the torque from DC motor equation
        """
        volts = control

        # Torque produced
        torque = self.model.kt.value * volts / self.model.R.value

        # Back EMF
        torque -= (self.model.kt.value**2) * dq / self.model.R.value

        return torque * torque_enable

    def to_mujoco(self):
        if self.vin == 0 or self.kp == 0:
            print(yellow(f"WARNING: kp or vin are not set"))

        kt = self.model.kt.value
        R = self.model.R.value

        kp = self.error_gain * self.kp * self.vin * self.max_pwm * kt / R
        damping = self.model.friction_viscous.value + kt**2 / R

        print_parameter("forcerange", self.vin * self.model.kt.value / R)
        print_parameter("armature", self.model.armature.value)
        print_parameter("kp", kp)
        print_parameter("damping", damping)
        print_parameter("frictionloss", self.model.friction_base.value)

        print("")

