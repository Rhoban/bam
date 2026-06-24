# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np


class Testbench:
    """Abstract base class for identification testbenches.

    A testbench defines the rigid-body dynamics of the physical setup:

    .. math::

        \\tau_m + \\tau_e(q) = M(q)\\ddot{q}

    where :math:`\\tau_m` is the motor torque, :math:`\\tau_e` the external
    (bias) torque, and :math:`M(q)` the effective inertia.
    """

    def compute_mass(self, q: float, dq: float) -> float:
        """Return the effective inertia at the current state [kg·m²].

        :param q: Joint angle [rad].
        :param dq: Joint velocity [rad/s].
        """
        raise NotImplementedError

    def compute_bias(self, q: float, dq: float) -> float:
        """Return the external (bias) torque at the current state [Nm].

        :param q: Joint angle [rad].
        :param dq: Joint velocity [rad/s].
        """
        raise NotImplementedError


class Pendulum(Testbench):
    """Single-axis pendulum testbench.

    A point mass at the tip of a uniform rod driven by the actuator output
    shaft.  The zero angle is the arm pointing downward; positive angles are
    counter-clockwise.

    :param log: Log dict containing ``"mass"`` [kg], ``"arm_mass"`` [kg], and
        ``"length"`` [m] keys.
    """

    def __init__(self, log: dict):
        self.mass = log["mass"]
        self.arm_mass = log["arm_mass"]
        self.length = log["length"]

    def compute_mass(self, q: float, dq: float) -> float:
        """Return the pendulum's rotational inertia about the pivot [kg·m²].

        :param q: Joint angle [rad] (unused — inertia is constant here).
        :param dq: Joint velocity [rad/s] (unused).
        """
        inertia = self.mass * self.length**2
        inertia += (self.arm_mass / 3) * self.length**2
        return inertia

    def compute_bias(self, q: float, dq: float) -> float:
        """Return the gravity torque at the current angle [Nm].

        :param q: Joint angle [rad].
        :param dq: Joint velocity [rad/s] (unused).
        """
        g = -9.80665
        return (self.mass + self.arm_mass / 2) * g * self.length * np.sin(q)
