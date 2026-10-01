# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

from typing import TYPE_CHECKING, Union

import numpy as np

from bam.actuator import Actuator, DCMotorActuator, TorchBackend
from bam.parameter import Parameter
from bam.testbench import Pendulum, Testbench
from bam.unitree.digital_servo import KD_STEP, KP_STEP, SPEED_STEP, TORQUE_STEP

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
        torque = (q_target - q) * self.kp * self.model.ratio.value + self.damping * (
            0.0 - dq
        )
        torque = self.backend.clamp(
            torque, -self.model.max_torque.value, self.model.max_torque.value
        )

        return torque

    def compute_torque(
        self,
        control: ArrayLike | None,
        torque_enable: bool,
        q: ArrayLike,
        dq: ArrayLike,
    ) -> ArrayLike:
        torques = control * torque_enable

        return torques

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


# ───────────────────────────────────────────────────────────────────────────
# Unitree J288 digital servo
# ───────────────────────────────────────────────────────────────────────────
#
# The J288 runs field-oriented current control with a hybrid PD law in its
# firmware. All protocol fields are rotor-side; here everything is output-side,
# using the exact gear ratio 70070/243. The firmware computes::
#
#     tau = tau_ff + kp * (p_des - q) + kd * (w_des - w_filtered)
#
# clamps it at a torque limit, and drives the current for it, which the supply
# voltage bounds once back-EMF builds up. ``w_filtered`` is the servo's own speed
# estimate, a first-order low-pass of the true speed (about 15 ms, identified).
# In mode 0 ("stop") the windings are shorted, which brakes the joint.
#
# The motor torque this actuator produces is what the servo reports in its state
# frame (motor-side, from the current, before gearbox losses). After each step it
# is available, rounded to the protocol's resolution, as
# :attr:`J288Actuator.reported_torque`, so a simulation built on it produces the
# same torque signal the real servo does.
#
# Commands other than the target position (``kd``, ``tau_ff``, ``w_des``, the
# mode) are attributes of the actuator, set by whoever drives it, the same way
# the mjlab backend writes ``kp`` and ``vin``. Logs recorded with per-step
# commands (a torque chirp, gains that change mid-log) are replayed by the
# actuator itself: :meth:`J288Actuator.load_log` stores the command columns and
# each :meth:`J288Actuator.compute_control` call consumes one step, which the
# stateful-actuator contract (one call per timestep, in order) guarantees lines
# up with the log.

#: Per-entry log fields replayed as commands, and the actuator attribute each sets.
COMMAND_FIELDS = {
    "kp": "kp",
    "kd": "kd",
    "goal_torque": "tau_ff",
    "goal_speed": "w_des",
}


class J288Pendulum(Pendulum):
    """Pendulum testbench with the load's own inertia.

    On short arms a compact mass is not a point: its inertia about its own
    centre (for a cylinder with its axis parallel to the pivot, m r^2 / 2) adds
    a few percent. The log may give it as ``mass_inertia`` [kg.m^2]; it is added
    to the rod and point-mass terms, and does not change the gravity torque.

    This is mainly important for short arm lengths and end weights that have a
    significant diameter.
    """

    def __init__(self, log: dict):
        super().__init__(log)
        self.mass_inertia = log.get("mass_inertia", 0.0)

    def compute_mass(self, q: float, dq: float) -> float:
        return super().compute_mass(q, dq) + self.mass_inertia


class J288Actuator(DCMotorActuator):
    """Unitree J288 digital servo, output side.

    Command attributes (floats, numpy arrays or torch tensors, broadcast with
    the state): ``kp`` [N.m/rad], ``kd`` [N.m.s/rad], ``tau_ff`` [N.m],
    ``w_des`` [rad/s], ``enabled`` (``False`` is mode 0), ``vin`` [V]. The
    target position is the ``q_target`` argument of :meth:`compute_control`.

    Identified parameters, created on the model:

    * ``kt`` [N.m/A]: fixed at Unitree's figure. With torque-based control only
      ``kt / R`` enters the dynamics, so ``R`` carries the fit; ``kt`` keeps the
      current estimate (used by the battery-drop model) in amps.
    * ``R`` [Ohm]: output-referred winding resistance.
    * ``ke`` [V/(rad/s)]: back-EMF constant.
    * ``torque_limit`` [N.m]: firmware clamp on the motor torque.
    * ``speed_filter_tau`` [s]: time constant of the firmware speed filter.
    * ``armature`` [kg.m^2]: reflected rotor inertia.

    :param testbench_class: Testbench class, normally :class:`J288Pendulum`.
    :param vin: Default supply voltage [V].
    :param kp: Default position gain [N.m/rad].
    :param kd: Default damping gain [N.m.s/rad].
    :param quantize: Round gains and commands to the protocol's resolution.
    """

    stateful = True

    def __init__(
        self,
        testbench_class: Testbench,
        vin: float = 25.2,
        kp: float = 0.97,
        kd: float = 0.02,
        quantize: bool = True,
    ):
        super().__init__(testbench_class, vin, kp)
        self.kd = kd
        self.tau_ff = 0.0
        self.w_des = 0.0
        self.enabled = True
        self.quantize = quantize

        # Firmware speed estimate, lazily initialized from the first velocity it
        # is given so it takes that value's type and shape (see reset()).
        self.w_filtered: ArrayLike | None = None
        self._to_reset_env_ids: list = []

        # Per-step command columns replayed from a log (see load_log()).
        self._schedule: dict[str, np.ndarray] | None = None
        self._schedule_step = 0

        # Outputs of the last compute_control(), for logging and observations.
        self.duty_cycle: ArrayLike | None = None
        self.reported_torque: ArrayLike | None = None

    # ── Parameters ──────────────────────────────────────────────────────────

    def initialize(self):
        # Only about half the motor torque reaches the output when
        # driving, so the motor-side load friction can exceed BAM's default
        # bound of 0.5.
        self.model.max_load_friction = 0.9

        self.model.kt = Parameter(0.554, 0.3, 1.0, optimize=False)
        self.model.R = Parameter(2.7, 1.0, 8.0)
        self.model.ke = Parameter(0.72, 0.55, 0.9)
        self.model.torque_limit = Parameter(1.0, 0.8, 1.2)
        self.model.speed_filter_tau = Parameter(0.015, 0.002, 0.05)
        self.model.armature = Parameter(7.8e-4, 1e-5, 3e-3)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def control_unit(self) -> str:
        return "N.m"

    # ── Backend helpers ─────────────────────────────────────────────────────
    # Operations bam.actuator.Backend does not provide, dispatched the same way:
    # on the backend the caller installed (mjlab installs TorchBackend).

    def _is_torch(self) -> bool:
        return isinstance(self.backend, TorchBackend)

    def _round_to(self, x: ArrayLike, step: float) -> ArrayLike:
        """Round to the nearest multiple of ``step``, as the protocol does."""
        if self._is_torch() and hasattr(x, "round"):
            return (x / step).round() * step
        return np.round(np.asarray(x) / step) * step

    def _where(self, mask, a: ArrayLike, b: ArrayLike) -> ArrayLike:
        """Elementwise ``a`` where ``mask`` else ``b``."""
        if self._is_torch():
            import torch

            like = next(x for x in (a, b) if isinstance(x, torch.Tensor))
            return torch.where(torch.as_tensor(mask, device=like.device).bool(), a, b)
        return np.where(mask, a, b)

    def _both(self, a, b, like: ArrayLike):
        """Elementwise ``a and b`` for bools, arrays or tensors."""
        if self._is_torch():
            import torch

            a = torch.as_tensor(a, device=like.device).bool()
            b = torch.as_tensor(b, device=like.device).bool()
            return a & b
        return np.logical_and(a, b)

    def braking_gain(self) -> float:
        """Damping of the shorted windings in mode 0, kt * ke / R [N.m.s/rad]."""
        return self.model.kt.value * self.model.ke.value / self.model.R.value

    # ── Log replay ──────────────────────────────────────────────────────────

    def load_log(self, log: dict):
        """Take the log's settings, and store its per-step command columns.

        Log-level ``kp``, ``kd`` and ``vin`` set the defaults. Any of the
        per-entry fields in :data:`COMMAND_FIELDS` (``kp``, ``kd``,
        ``goal_torque``, ``goal_speed``) are replayed step by step, delayed by
        the model's ``command_delay`` like the goal position.
        """
        super().load_log(log)
        self.kd = log.get("kd", 0.0)
        self.tau_ff = 0.0
        self.w_des = 0.0
        self.enabled = True

        entries = log.get("entries", [])
        present = [f for f in COMMAND_FIELDS if entries and f in entries[0]]
        if not present:
            self._schedule = None
            return

        delay = getattr(self.model, "command_delay", None)
        delay = 0.0 if delay is None else delay.value
        dt = float(np.asarray(log.get("dt", 0.0)).reshape(-1)[0])
        self._schedule = {}
        for field in present:
            column = np.stack([np.asarray(e[field], dtype=float) for e in entries])
            if delay > 0.0 and dt > 0.0:
                # Imported here: bam.simulate imports the model registry.
                from bam.simulate import fractional_delay_shift

                column = fractional_delay_shift(column, delay, dt)
            self._schedule[field] = column
        self._schedule_step = 0

    def _apply_schedule(self) -> None:
        if self._schedule is None:
            return
        k = self._schedule_step
        for field, column in self._schedule.items():
            setattr(self, COMMAND_FIELDS[field], column[min(k, len(column) - 1)])
        self._schedule_step += 1

    # ── Firmware state ──────────────────────────────────────────────────────

    def reset(self, env_ids=...) -> None:
        """Reset the firmware speed filter.

        The filter state is a velocity, only known when the control law runs,
        so the reset is deferred to the next :meth:`compute_control`, which
        re-seeds the selected environments from the velocity it is given. The
        replay position of a loaded log is not affected.
        """
        if env_ids is ...:
            self.w_filtered = None
            self._to_reset_env_ids.clear()
        else:
            self._to_reset_env_ids.append(env_ids)

    def get_state(self):
        return self.w_filtered, list(self._to_reset_env_ids)

    def set_state(self, state) -> None:
        w_filtered, to_reset = (None, []) if state is None else state
        self.w_filtered = w_filtered
        self._to_reset_env_ids = list(to_reset)

    def _filter_speed(self, dq: ArrayLike, dt: float) -> ArrayLike:
        w = self.w_filtered
        if w is None:
            w = dq * 1.0  # a copy, in dq's type
            self._to_reset_env_ids.clear()
        elif self._to_reset_env_ids:
            for env_ids in self._to_reset_env_ids:
                w[env_ids] = dq[env_ids]
            self._to_reset_env_ids.clear()
        # dt is a float, or a per-log array when BAM rolls out a batch of logs.
        dt = np.asarray(dt, dtype=float)
        if np.any(dt > 0.0):
            a = 1.0 - np.exp(-dt / self.model.speed_filter_tau.value)
            w = w + (float(a) if a.ndim == 0 else a) * (dq - w)
        self.w_filtered = w
        return w

    # ── Control law ─────────────────────────────────────────────────────────

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> ArrayLike:
        """Firmware torque for the current commands, as the servo reports it.

        Must be called once per timestep, in order (see ``stateful``).

        :param q_target: Target position [rad].
        :param q: Position [rad].
        :param dq: Velocity [rad/s].
        :param dt: Timestep [s].
        :returns: Motor torque [N.m], clamped by the torque limit and the
            supply voltage.
        """
        self._apply_schedule()

        kp, kd, tau_ff, w_des = self.kp, self.kd, self.tau_ff, self.w_des
        if self.quantize:
            kp = self._round_to(kp, KP_STEP)
            kd = self._round_to(kd, KD_STEP)
            tau_ff = self._round_to(tau_ff, TORQUE_STEP)
            w_des = self._round_to(w_des, SPEED_STEP)

        w_filtered = self._filter_speed(dq, dt)
        torque = tau_ff + kp * (q_target - q) + kd * (w_des - w_filtered)

        limit = self.model.torque_limit.value
        torque = self.backend.clamp(torque, -limit, limit)

        # The current for that torque needs R * I + ke * dq volts, at most vin.
        kt, R, ke = self.model.kt.value, self.model.R.value, self.model.ke.value
        g = kt / R
        torque = self.backend.clamp(
            torque, g * (-self.vin - ke * dq), g * (self.vin - ke * dq)
        )

        self.duty_cycle = (torque / g + ke * dq) / self.vin
        return torque

    def compute_torque(
        self,
        control: ArrayLike | None,
        torque_enable: bool,
        q: ArrayLike,
        dq: ArrayLike,
    ) -> ArrayLike:
        """Motor torque: the control torque in mode 1, shorted-winding braking
        in mode 0 (``torque_enable`` false, or ``enabled`` false).

        Also sets :attr:`reported_torque`, the value the servo would report.
        """
        braking = -self.braking_gain() * dq
        if control is None:
            torque = braking
        else:
            mask = self._both(torque_enable, self.enabled, like=control)
            torque = self._where(mask, control, braking)
        self.reported_torque = (
            self._round_to(torque, TORQUE_STEP) if self.quantize else torque
        )
        return torque
