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
        raise NotImplementedError


class LiftAndDrop(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        keyframes = [[0.0, 0.2, 0.0], [2.0, 0.1545, 0.0]]
        l = cubic_interpolate(keyframes, t)

        enable = t < 2.0
 
        return l, enable


class SinusTimeSquare(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        l = 0.2045 + 0.05* np.sin(t**2)

        return l, True


class UpAndDown(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        keyframes = [
            [0.0, 0.0, 0.0],
            [2.0, 0.1545, 0.0],
            [6.0, 0., 0.0],
        ]
        l = cubic_interpolate(keyframes, t)

        return l, True

class SinSin(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        l = 0.2 + np.sin(t) * 0.05 + np.sin(5.0 * t) * 0.01 * np.sin(t*2.0)

        return l, True


class Nothing(Trajectory):
    duration = 6.0

    def __call__(self, t: float):
        return 0.2, False


trajectories = {
    "lift_and_drop": LiftAndDrop(),
    "sin_time_square": SinusTimeSquare(),
    "up_and_down": UpAndDown(),
    "sin_sin": SinSin(),
    "nothing": Nothing(),
}
