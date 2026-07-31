# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from copy import copy
from .model import Model


def fractional_delay_shift(goal: np.ndarray, delay: float, dt: float) -> np.ndarray:
    """Shift a goal sequence backward in time by ``delay`` seconds.

    The shift is fractional: linear interpolation between the two neighbouring
    samples, so a delay that is not an integer number of timesteps is honoured.
    Indices that fall before the start are held at the first sample. This is the
    single implementation of the command-delay shift, shared by the reference,
    MuJoCo and mjlab simulators so they stay consistent.

    :param goal: Array whose first axis is time, shape ``(n_steps, ...)``. Any
        trailing axes (e.g. a batch/environment axis) are carried through untouched.
    :param delay: Delay in seconds (``>= 0``).
    :param dt: Timestep in seconds.
    :returns: Array of the same shape as ``goal``, delayed in time.
    """
    n = goal.shape[0]
    d = delay / dt
    d0 = int(np.floor(d))
    frac = d - d0
    idx = np.arange(n)
    i0 = np.clip(idx - d0, 0, n - 1)
    i1 = np.clip(idx - d0 - 1, 0, n - 1)
    return (1.0 - frac) * goal[i0] + frac * goal[i1]


class Simulator:
    """Single-axis pendulum simulator used during identification.

    Implements the BAM simulation loop: firmware control law → motor torque →
    friction budget → stopping-torque clipping → Euler integration.  Used by
    ``bam.fit`` to roll out a model against recorded logs and compute the
    identification loss.

    :param model: BAM friction model to simulate.
    """

    def __init__(self, model: Model):
        self.screen = None
        self.model = model
        self.reset()

    def reset(self, q: float = 0.0, dq: float = 0.0):
        """
        Resets the simulation to a given state
        """
        self.q = copy(q)
        self.dq = copy(dq)
        self.t = 0.0

        self.model.reset()

    def step(self, control: None | float, torque_enable: bool, dt: float):
        """Advance the simulation by one timestep.

        :param control: Control signal sent to the actuator (voltage, current,
            or torque depending on the actuator type). ``None`` is treated as zero.
        :param torque_enable: Whether the actuator is powered. When ``False``
            the motor torque is zero and only gravity and friction act.
        :param dt: Timestep [s].
        """
        bias_torque = self.model.actuator.testbench.compute_bias(
            self.q + self.model.q_offset.value, self.dq
        )
        motor_torque = self.model.actuator.compute_torque(
            control, torque_enable, self.q + self.model.q_offset.value, self.dq
        )
        frictionloss, damping = self.model.compute_frictions(
            motor_torque, bias_torque, self.dq
        )

        inertia = (
            self.model.actuator.testbench.compute_mass(
                self.q + self.model.q_offset.value, self.dq
            )
            + self.model.actuator.get_extra_inertia()
        )
        net_torque = motor_torque + bias_torque

        # Tau_stop is the torque required to stop the motor (reach a velocity of 0 after dt)
        tau_stop = (inertia / dt) * self.dq + net_torque
        static_friction = -np.sign(tau_stop) * np.min(
            [np.abs(tau_stop), frictionloss + damping * np.abs(self.dq)], axis=0
        )
        net_torque += static_friction

        angular_acceleration = net_torque / inertia

        self.dq += angular_acceleration * dt
        self.dq = np.clip(self.dq, -100.0, 100.0)
        self.q += self.dq * dt
        self.t += dt

    def rollout_log(
        self, log: dict, reset_period: float = None, simulate_control: bool = False
    ):
        """Roll out the model against a recorded log and return predicted trajectories.

        :param log: Processed log dict as returned by :meth:`bam.logs.Logs.make_batch`
            or loaded directly from a processed JSON file.
        :param reset_period: If set, re-synchronize the simulator state to the
            log at this interval [s]. Useful when error accumulation destabilizes
            long rollouts.
        :param simulate_control: If ``True``, recompute the control signal from
            the simulated state using the firmware control law. If ``False``,
            use the control values recorded in the log.
        :returns: Tuple ``(positions, velocities, controls)`` — lists of values
            at each timestep.
        """
        positions = []
        velocities = []
        controls = []

        reset_period_t = 0.0
        dt = log["dt"]
        first_entry = log["entries"][0]
        self.reset(
            first_entry["position"],
            first_entry["speed"] if "speed" in first_entry else 0.0,
        )
        self.model.actuator.load_log(log)

        # Command delay: a rig-level transport lag between the commanded goal
        # position and the actuator response. The goal sequence is known in advance
        # (log data), so the delayed goal is precomputed once by fractionally
        # shifting the sequence (linear interpolation between the two neighbouring
        # samples). command_delay is a base Model parameter (always present); when
        # it is 0 this reduces to the original, unshifted goal. Works for scalar
        # (per-log) and vector (batched) goals, broadcasting over the batch axis.
        cmd_delay = getattr(self.model, "command_delay", None)
        delayed_goal = None
        if cmd_delay is not None and cmd_delay.value > 0.0 and simulate_control:
            # Scalar timestep for the delay index math (dt is a per-log vector in
            # batched rollouts; all logs share the same dt).
            dt_scalar = float(np.asarray(dt).reshape(-1)[0])
            goal = np.stack([e["goal_position"] for e in log["entries"]])
            delayed_goal = fractional_delay_shift(goal, cmd_delay.value, dt_scalar)

        for k, entry in enumerate(log["entries"]):
            reset_period_t += dt
            if reset_period is not None and reset_period_t > reset_period:
                reset_period_t = 0.0
                self.reset(entry["position"], entry["speed"])
            positions.append(copy(self.q))
            velocities.append(copy(self.dq))

            if simulate_control:
                goal_k = (
                    delayed_goal[k]
                    if delayed_goal is not None
                    else entry["goal_position"]
                )
                control = self.model.actuator.compute_control(
                    goal_k, self.q, self.dq, dt
                )
            else:
                if "control" in entry:
                    control = entry["control"]
                else:
                    control = self.model.actuator.compute_control(
                        entry["goal_position"], entry["position"], self.dq, dt
                    )

            controls.append(copy(control))

            self.step(control, entry["torque_enable"], dt)

        return positions, velocities, controls
