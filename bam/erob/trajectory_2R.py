import placo
import numpy as np


class Trajectory:
    def __init__(self):
        self.duration = 0.0

    def __call__(self):
        pass


class SquareWave:
    def __init__(
        self, z_min=-0.3, z_max=-0.2, x_range=0.5, strides=5, stride_duration=8.0
    ):

        self.spline = placo.CubicSpline3D()
        t = 0
        stride_length = x_range / strides
        for k in range(strides):
            entries = [
                [-(x_range / 2) + k * stride_length, 0, z_min],
                [-(x_range / 2) + k * stride_length, 0, z_max],
                [-(x_range / 2) + (k + 0.5) * stride_length, 0, z_max],
                [-(x_range / 2) + (k + 0.5) * stride_length, 0, z_min],
            ]

            for n_entry, entry in enumerate(entries):
                self.spline.add_point(
                    t + n_entry * stride_duration / len(entries),
                    np.array(entry),
                    np.array([0.0, 0.0, 0.0]),
                )
                self.spline.add_point(
                    t + (n_entry + 0.5) * stride_duration / len(entries),
                    np.array(entry),
                    np.array([0.0, 0.0, 0.0]),
                )

            t += stride_duration

        self.duration = t

    def __call__(self, t):
        return self.spline.pos(t)


class TriangularWave:
    def __init__(
        self, z_min=-0.3, z_max=-0.2, x_range=0.5, strides=5, stride_duration=8.0
    ):

        self.spline = placo.CubicSpline3D()
        t = 0
        stride_length = x_range / strides
        for k in range(strides):
            entries = [
                [-(x_range / 2) + k * stride_length, 0, z_min],
                [-(x_range / 2) + (k + 0.5) * stride_length, 0, z_max],
            ]
            if k == strides - 1:
                entries.append([-(x_range / 2) + (k + 1) * stride_length, 0, z_min])

            for n_entry, entry in enumerate(entries):
                self.spline.add_point(
                    t + n_entry * stride_duration / len(entries),
                    np.array(entry),
                    np.array([0.0, 0.0, 0.0]),
                )
                self.spline.add_point(
                    t + (n_entry + 0.5) * stride_duration / len(entries),
                    np.array(entry),
                    np.array([0.0, 0.0, 0.0]),
                )

            t += stride_duration

        self.duration = t

    def __call__(self, t):
        return self.spline.pos(t)


class Square:
    def __init__(
        self,
        x_offset=0.0,
        z_offset=-0.3,
        range=0.2,
        side_duration=2.0,
        pause_duration=1.0,
    ):

        self.spline = placo.CubicSpline3D()

        entries = [
            [-(range / 2) + x_offset, 0, z_offset - range / 2],
            [-(range / 2) + x_offset, 0, z_offset + range / 2],
            [(range / 2) + x_offset, 0, z_offset + range / 2],
            [(range / 2) + x_offset, 0, z_offset - range / 2],
            [-(range / 2) + x_offset, 0, z_offset - range / 2],
        ]

        t = 0
        for entry in entries:
            self.spline.add_point(
                t,
                np.array(entry),
                np.array([0.0, 0.0, 0.0]),
            )

            self.spline.add_point(
                t + pause_duration,
                np.array(entry),
                np.array([0.0, 0.0, 0.0]),
            )
            t += pause_duration + side_duration

        self.duration = t

    def __call__(self, t):
        return self.spline.pos(t)


class Circle:
    def __init__(
        self,
        x_offset=0.0,
        z_offset=-0.3,
        range=0.2,
        duration=8.0,
    ):
        self.spline = placo.CubicSpline3D()
        self.x_offset = x_offset
        self.z_offset = z_offset
        self.range = range
        self.duration = duration

    def __call__(self, t):
        x = self.x_offset + (np.sin(2 * np.pi * t / self.duration) * self.range / 2.0)
        z = self.z_offset + (np.cos(2 * np.pi * t / self.duration) * self.range / 2.0)

        return np.array([x, 0, z])
