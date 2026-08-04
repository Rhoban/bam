# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING, Union
from bam.message import yellow, print_parameter, bright
from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum

if TYPE_CHECKING:
    from bam.actuator import ArrayLike


class STS3215Actuator(VoltageControlledActuator):
    """
    Feetech STS3215 7.4v

    The firmware rate-limits the target position it feeds to its P controller,
    which makes :meth:`compute_control` stateful (see :attr:`q_target_smooth`).
    """

    stateful = True

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.4,
            kp=32,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and STS3215 actuators
            # here, firmware_kp = kp
            error_gain=0.166,
            # self.error_gain = 0.001 * np.rad2deg(1.0)
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=0.97,  # TODO, but can we assume 1.0 ?
        )

        self.default_max_velocity = (3400 * 2 * np.pi) / 4096

        # Firmware's internal target position, rate-limited to max_velocity.
        # It is lazily initialized on the first compute_control() call so that it
        # adopts the type and shape of the state it is fed (float, numpy array or
        # torch tensor), and does not require a log to be loaded.
        self.q_target_smooth: ArrayLike | None = None

        # Environments whose internal target still has to be re-seeded, see reset()
        self._to_reset_env_ids: list = []

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def reset(self, env_ids=...) -> None:
        """Reset the firmware's internal (rate-limited) target position.

        That target is a joint position, which is only known when the controller
        runs: the reset is therefore deferred to the next :meth:`compute_control`,
        which re-seeds the selected environments from the position it is given.

        :param env_ids: Environments to reset, ``...`` (default) for all of them.
        """
        if env_ids is ...:
            self.q_target_smooth = None
            self._to_reset_env_ids.clear()
        else:
            self._to_reset_env_ids.append(env_ids)

    def get_state(self):
        # Internal target position, plus the resets that are still pending
        return self.q_target_smooth, list(self._to_reset_env_ids)

    def set_state(self, state) -> None:
        q_target_smooth, to_reset_env_ids = (None, []) if state is None else state
        self.q_target_smooth = q_target_smooth
        self._to_reset_env_ids = list(to_reset_env_ids)

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.784532, 0.05, 2.5)  # docs says 8 kg.cm / A

        self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 0.1, 10.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0001, 0.00001, 0.04)

        self.model.q_offset = Parameter(0, -0.2, 0.2)

        self.model.max_velocity = Parameter(
            self.default_max_velocity,
            0.1 * self.default_max_velocity,
            10.0 * self.default_max_velocity,
        )

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> ArrayLike | None:
        """Compute the voltage command from the rate-limited position error.

        The firmware does not track ``q_target`` directly: it moves an internal
        target towards it at most at ``max_velocity``, and runs its P controller
        on that internal target. This is stateful, so :meth:`compute_control`
        must be called once per timestep, in order.

        The state is kept in ``self.q_target_smooth`` and is (re)initialized to
        the current position ``q`` on the first call after a :meth:`reset` — the
        way the servo behaves when it powers on. It therefore inherits the type
        and shape of ``q`` and works on any backend (float, numpy array or torch
        tensor, in the latter case vectorized over all environments).

        :param q_target: Target joint angle(s) [rad].
        :param q: Current joint angle(s) [rad].
        :param dq: Current joint velocity(ies) [rad/s] (unused here).
        :param dt: Timestep [s], used by the internal target rate limiter.
        :returns: Voltage [V] sent to the motor.
        """
        q_target_smooth = self.q_target_smooth
        if q_target_smooth is None:
            # First call, or first call after a full reset
            q_target_smooth = q
            self._to_reset_env_ids.clear()
        elif self._to_reset_env_ids:
            # Environments reset since the last call: their internal target has to
            # follow their (teleported) position
            for env_ids in self._to_reset_env_ids:
                q_target_smooth[env_ids] = q[env_ids]
            self._to_reset_env_ids.clear()

        # Internal target position is clipped using maximum velocity
        max_step = self.model.max_velocity.value * dt
        self.q_target_smooth = self.backend.clamp(
            q_target,
            q_target_smooth - max_step,
            q_target_smooth + max_step,
        )

        duty_cycle = (
            (self.q_target_smooth - q)
            * self.kp
            * self.error_gain
            * self.model.error_gain_ratio.value
        )
        duty_cycle = self.backend.clamp(duty_cycle, -self.max_pwm, self.max_pwm)
        self.duty_cycle = duty_cycle  # for logging (and the battery drop model)

        return self.vin * duty_cycle
