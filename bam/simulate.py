# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from copy import copy
from .model import Model


class Simulator:
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
        """
        Steps the simulation for dt given the applied control
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
        self.q += self.dq * dt + 0.5 * angular_acceleration * dt**2
        self.t += dt

    def rollout_log(
        self, log: dict, reset_period: float = None, simulate_control: bool = False
    ):
        """
        Read a given log dict and return the sequential reached positions
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

        for entry in log["entries"]:
            reset_period_t += dt
            if reset_period is not None and reset_period_t > reset_period:
                reset_period_t = 0.0
                self.reset(entry["position"], entry["speed"])
            positions.append(copy(self.q))
            velocities.append(copy(self.dq))

            if simulate_control:
                control = self.model.actuator.compute_control(
                    entry["goal_position"], self.q, self.dq, dt
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
