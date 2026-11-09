# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import math

import numpy as np

from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench


class MD300Actuator(VoltageControlledActuator):
    """One output of the MangDang MD-300-150 in voltage (PWM) position control.

    Each output is a coreless brushed DC motor with a gearbox, driven by a
    position PD loop on an AT32 microcontroller (``control_mode`` 1)::

        duty = clip(pwm_scale * kp * (error - dead band) - D, +/-max_pwm)

    with the error in degrees, a 0.1 deg dead band (errors under it count as
    zero) and the D term proportional to the measured velocity, not to the
    error rate. ``pwm_scale``, ``kp``, ``kd``, ``max_pwm`` and the supply
    voltage are read from each log; ``error_gain_ratio`` and ``kd_ratio`` scale
    the P and D gains.

    The plant departs from :class:`~bam.actuator.VoltageControlledActuator` in
    two ways: the back-EMF constant ``kt_emf`` is separate from the torque
    constant ``kt``, and while torque is off the H-bridge still brakes the motor
    (``-kt * kt_emf * dq / R``). ``error_gain_ratio`` and ``kt_emf`` are
    effective values: they also absorb loop lag and viscous damping.
    """

    #: D gain [duty per (rad/s) per unit kd] at ``pwm_scale`` 0.001, measured
    #: with constant-velocity ramps; the firmware's D term scales with
    #: ``pwm_scale``.
    KD_GAIN: float = 2.87e-5
    #: Firmware dead band on the position error [deg].
    DEADBAND_DEG: float = 0.1
    #: Identified configuration, used when no log is loaded (MuJoCo, mjlab).
    DEFAULT_PWM_SCALE: float = 0.001
    DEFAULT_MAX_PWM: float = 0.99
    DEFAULT_KD: float = 400.0

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=80.0,
            error_gain=self.DEFAULT_PWM_SCALE * 180.0 / math.pi,
            max_pwm=self.DEFAULT_MAX_PWM,
        )
        self.kd = self.DEFAULT_KD
        self.kd_scale = 1.0
        self.deadband = math.radians(self.DEADBAND_DEG)

    def load_log(self, log: dict):
        """Read the firmware configuration from the log.

        Values may be scalars or arrays (batched logs), so nothing is cast.
        The frame's current cap clips the loop output before the duty scale,
        so it acts as a duty cap ``pwm_scale * cur_cap_ma``.
        """
        super().load_log(log)
        scale = log.get("at32_pwm_scale", self.DEFAULT_PWM_SCALE)
        self.error_gain = scale * (180.0 / math.pi)
        self.kd_scale = scale / self.DEFAULT_PWM_SCALE
        max_pwm = log.get("at32_max_pwm_duty_cycle", self.DEFAULT_MAX_PWM)
        cap = log.get("cur_cap_ma")
        self.max_pwm = max_pwm if cap is None else np.minimum(max_pwm, cap * scale)
        self.kd = log.get("at32_kd_position", log.get("kd", self.DEFAULT_KD))
        supply = log.get("supply_v")
        if supply is not None:
            self.vin = supply

    def initialize(self):
        # Torque constant at the output [Nm/A]
        self.model.kt = Parameter(0.68, 0.40, 1.00)
        # Back-EMF constant [V.s/rad]
        self.model.kt_emf = Parameter(0.6, 0.25, 1.2)
        # Winding resistance [Ohm]
        self.model.R = Parameter(11.0, 8.0, 24.0)
        # Apparent inertia at the output [kg m^2]
        self.model.armature = Parameter(3e-4, 1e-5, 5e-3)
        # Scales on the firmware P and D gains
        self.model.error_gain_ratio = Parameter(1.0, 0.7, 1.6)
        self.model.kd_ratio = Parameter(1.0, 0.0, 2.0)

    def compute_control(self, q_target, q, dq, dt):
        error = q_target - q
        error = error - self.backend.clamp(error, -self.deadband, self.deadband)
        duty = error * self.kp * self.error_gain * self.model.error_gain_ratio.value
        duty = (
            duty
            - self.kd * self.KD_GAIN * self.kd_scale * self.model.kd_ratio.value * dq
        )
        duty = self.backend.clamp(duty, -self.max_pwm, self.max_pwm)
        self.duty_cycle = duty
        return self.vin * duty

    def compute_torque(self, control, torque_enable, q, dq):
        kt = self.model.kt.value
        R = self.model.R.value
        current = (control - self.model.kt_emf.value * dq) / R
        powered = kt * current * torque_enable
        brake = -kt * self.model.kt_emf.value * dq / R
        return powered + brake * (1 - torque_enable)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
