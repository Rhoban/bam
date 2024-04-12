import numpy as np


def cubic_interpolate(keyframes: list, t: float):
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
    duration = None

    def __call__(self, t: float):
        """
        Retrieve (angle, torque_enable) at time t
        """
        raise NotImplementedError


class LiftAndDrop(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        keyframes = [[0.0, 0.0, 0.0], [2.0, -np.pi / 2, 0.0]]
        angle = cubic_interpolate(keyframes, t)

        enable = t < 2.0

        return angle, enable


class SinusTimeSquare(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        angle = np.sin(t**2)

        return angle, True


class UpAndDown(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        keyframes = [
            [0.0, 0.0, 0.0],
            [3.0, np.pi / 2, 0.0],
            [6.0, 0.8 * np.pi / 2, 0.0],
        ]
        angle = cubic_interpolate(keyframes, t)

        return angle, True


class SinSin(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        angle = np.sin(t) * np.pi / 2 + np.sin(5.0 * t) * 0.5 * np.sin(t*2.0)

        return angle, True


class Nothing(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        return 0.0, False


trajectories = {
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
