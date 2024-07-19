import placo
import numpy as np


class Trajectory:
    def __init__(self):
        self.duration = 0.0

    def __call__(self):
        pass


class SquareWave:
    def __init__(
        self, z_min=-0.35, z_max=-0.25, x_range=0.5, strides=8, stride_duration=5.0
    ):

        self.spline = placo.CubicSpline3D()
        t = 0
        stride_length = x_range / strides
        for k in range(strides):
            self.spline.add_point(
                t,
                np.array([-(x_range / 2) + k * stride_length, 0, z_min]),
                np.array([0.0, 0.0, 0.0]),
            )
            self.spline.add_point(
                t + stride_duration / 4,
                np.array([-(x_range / 2) + k * stride_length, 0, z_max]),
                np.array([0.0, 0.0, 0.0]),
            )
            self.spline.add_point(
                t + 2 * stride_duration / 4,
                np.array([-(x_range / 2) + (k + 0.5) * stride_length, 0, z_max]),
                np.array([0.0, 0.0, 0.0]),
            )
            self.spline.add_point(
                t + 3 * stride_duration / 4,
                np.array([-(x_range / 2) + (k + 0.5) * stride_length, 0, z_min]),
                np.array([0.0, 0.0, 0.0]),
            )
            t += stride_duration

        self.duration = t

    def __call__(self, t):
        return self.spline.pos(t)
