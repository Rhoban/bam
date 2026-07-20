# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np


def cubic_interpolate(keyframes: list, t: float) -> float:
    """Interpolate a scalar signal through a list of keyframes with cubic splines.

    Each keyframe is a triplet ``(t, x, x')`` where ``t`` is the time, ``x``
    the value, and ``x'`` the desired first derivative at that point.

    :param keyframes: List of ``[t, x, dx/dt]`` triplets, sorted by time.
    :param t: Query time.
    :returns: Interpolated value at time ``t``.
    """
    if t < keyframes[0][0]:
        return keyframes[0][1]
    if t > keyframes[-1][0]:
        return keyframes[-1][1]

    for i in range(len(keyframes) - 1):
        if keyframes[i][0] <= t <= keyframes[i + 1][0]:
            t0, x0, x0p = keyframes[i]
            t1, x1, x1p = keyframes[i + 1]

            A = [
                [1, t0, t0**2, t0**3],
                [0, 1, 2 * t0, 3 * t0**2],
                [1, t1, t1**2, t1**3],
                [0, 1, 2 * t1, 3 * t1**2],
            ]
            b = [x0, x0p, x1, x1p]
            w = np.linalg.solve(A, b)

            return w[0] + w[1] * t + w[2] * t**2 + w[3] * t**3


class Trajectory:
    """Abstract base class for identification trajectories.

    A trajectory is a callable that maps a time ``t`` to a target angle and a
    torque-enable flag.  All built-in trajectories run for 6 seconds.
    """

    duration: float = None

    def __call__(self, t: float) -> tuple[float, bool]:
        """Return ``(angle [rad], torque_enable)`` at time ``t``.

        :param t: Time since the start of the trajectory [s].
        """
        raise NotImplementedError


class LiftAndDrop(Trajectory):
    """Cubic move to −π/2 over 2 s, then torque disabled (gravity drop).

    Useful for identifying backdrivability and Stribeck effects at very low
    speed, as the arm falls freely under gravity after the motor is released.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        keyframes = [[0.0, 0.0, 0.0], [2.0, -np.pi / 2, 0.0]]
        angle = cubic_interpolate(keyframes, t)
        enable = t < 2.0
        return angle, enable


class SinusTimeSquare(Trajectory):
    """Progressively faster sinusoidal trajectory: :math:`\\sin(t^2)`.

    General-purpose trajectory that sweeps a wide velocity range in a single
    run.  Recommended as the primary identification trajectory.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return np.sin(t**2), True


class UpAndDown(Trajectory):
    """Slow cubic path 0 → π/2 → 0.8·π/2.

    Emphasises static friction and load-dependent effects at low to medium
    speed.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        keyframes = [
            [0.0, 0.0, 0.0],
            [3.0, np.pi / 2, 0.0],
            [6.0, 0.8 * np.pi / 2, 0.0],
        ]
        return cubic_interpolate(keyframes, t), True

class HalfSine(Trajectory):
    """Slow half-sine path 0 → π/2.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return np.sin(t/2) * np.pi/2, True
    
class Steps(Trajectory):
    """Steps
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        stages = [
            [0, 0],
            [1, 0.75],
            [2.5, 1.55],
            [4, 0.75],
            [5, 0],
            [6, 0]
        ]
        for i in range(len(stages) - 1):
            if stages[i][0] <= t <= stages[i + 1][0]:
                return stages[i][1], True

class SinSin(Trajectory):
    """Multi-frequency trajectory: :math:`\\sin(t)\\cdot\\pi/2 + \\sin(5t)\\cdot 0.5\\cdot\\sin(2t)`.

    Rich spectral content covers a broad range of velocities and accelerations.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        angle = np.sin(t) * np.pi / 2 + np.sin(5.0 * t) * 0.5 * np.sin(t * 2.0)
        return angle, True


class Nothing(Trajectory):
    """Zero torque for the full duration (pure gravity response).

    Useful to isolate backdrivability and measure passive dynamics.
    """

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        return 0.0, False


#: Registry of all built-in trajectories, keyed by name.
#: Pass one of these names to the ``--trajectory`` argument of the recording scripts.
trajectories: dict[str, Trajectory] = {
    "lift_and_drop": LiftAndDrop(),
    "sin_time_square": SinusTimeSquare(),
    "up_and_down": UpAndDown(),
    "sin_sin": SinSin(),
    "nothing": Nothing(),
}

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    trajectory = LiftAndDrop()

    ts = np.linspace(0.0, 5.0, 1000)
    xs = [trajectory(t)[0] for t in ts]
    en = [trajectory(t)[1] for t in ts]

    plt.plot(ts, xs, label="Angle")
    plt.fill_between(
        ts, min(xs), max(xs), en, alpha=0.2, color="green", label="Torque enable"
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid()
    plt.show()
