# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""MD-300-150 trajectories beyond BAM's generic set.

Small-signal trajectories for the abduction output. The generic BAM
trajectories swing the pendulum up to about +/-1.8 rad; on the robot the
abduction joint mostly holds the leg with small, fast corrections and at most
about +/-0.35 rad of range. These trajectories stay within +/-0.35 rad of the
vertical, so a payload sets a gravity bias of up to ``m g l sin(0.35)`` and the
motion dwells where static friction matters.

Recorded target streams (``gait_hip``, ``gait_knee``): the setpoints a walking
gait sends to one joint, held for 20 ms each as a 50 Hz controller issues them,
played about a gravity bias (:class:`TargetStream`).

Importing this module adds them to :data:`bam.trajectory.trajectories`,
so ``bam.mangdang.record --trajectory abd_steps`` works like any built-in name.
"""

from pathlib import Path

import numpy as np

from bam.trajectory import SinusTimeSquare, Trajectory, cubic_interpolate, trajectories

#: Largest excursion from the vertical [rad] (the robot's abduction range).
AMPLITUDE = 0.35


class AbdSteps(Trajectory):
    """Held step targets on both sides: 0, 0.05, 0.15, 0.35 rad and back.

    Policy-like step targets of three sizes, each held about 0.55 s, so the
    joint settles against the gravity bias of each level on both sides.
    """

    duration = 6.0
    levels = (
        0.0,
        0.05,
        0.15,
        AMPLITUDE,
        0.15,
        0.0,
        -0.05,
        -0.15,
        -AMPLITUDE,
        -0.15,
        0.0,
    )

    def __call__(self, t: float) -> tuple[float, bool]:
        k = min(int(t / (self.duration / len(self.levels))), len(self.levels) - 1)
        return self.levels[k], True


class AbdSinTimeSquare(Trajectory):
    """``sin_time_square`` scaled to +/-0.35 rad (up to about 4 rad/s)."""

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        angle, enable = SinusTimeSquare()(t)
        return AMPLITUDE * angle, enable


class AbdSweep(Trajectory):
    """Hold at +0.25 rad with a 0.05 rad sine sweeping 0.5 -> 4 Hz, then mirror.

    The hold sets a steady gravity load and the sweep probes the small-signal
    response around it; the mirror at -0.25 rad reverses the load. The sine
    amplitude ramps in and out over 0.2 s so the path is continuous.
    """

    duration = 6.0
    offset = 0.25
    amplitude = 0.05
    f0, f1 = 0.5, 4.0
    #: (start, end) of the two sweeps; cubic moves fill the gaps.
    windows = ((0.5, 2.75), (3.25, 5.5))
    taper = 0.2

    def _sweep(self, tau: float, length: float) -> float:
        phase = (
            2.0
            * np.pi
            * (self.f0 * tau + (self.f1 - self.f0) * tau**2 / (2.0 * length))
        )
        envelope = min(1.0, tau / self.taper, (length - tau) / self.taper)
        return self.amplitude * envelope * np.sin(phase)

    def __call__(self, t: float) -> tuple[float, bool]:
        (a0, a1), (b0, b1) = self.windows
        if t < a0:
            return cubic_interpolate([[0.0, 0.0, 0.0], [a0, self.offset, 0.0]], t), True
        if t < a1:
            return self.offset + self._sweep(t - a0, a1 - a0), True
        if t < b0:
            return cubic_interpolate(
                [[a1, self.offset, 0.0], [b0, -self.offset, 0.0]], t
            ), True
        if t < b1:
            return -self.offset + self._sweep(t - b0, b1 - b0), True
        return cubic_interpolate(
            [[b1, -self.offset, 0.0], [self.duration, 0.0, 0.0]], t
        ), True


class AbdDrop(Trajectory):
    """Lift to +0.35 rad in 1.5 s, hold, release at 2 s: a small free swing."""

    duration = 6.0

    def __call__(self, t: float) -> tuple[float, bool]:
        angle = cubic_interpolate([[0.0, 0.0, 0.0], [1.5, AMPLITUDE, 0.0]], t)
        return angle, t < 2.0


SMALL_SIGNAL = {
    "abd_steps": AbdSteps(),
    "abd_sts": AbdSinTimeSquare(),
    "abd_sweep": AbdSweep(),
    "abd_drop": AbdDrop(),
}

#: Directory of the recorded streams (``t_s,goal_rad`` CSV, ``#`` comments).
STREAMS_DIR = Path(__file__).resolve().parent / "streams"


class TargetStream(Trajectory):
    """A recorded joint-target stream, played as a zero-order hold.

    The file holds one target per ``period`` about the stream's mean. After a
    cubic lead-in from 0 to ``offset`` plus the first target, each target is
    held for one period, as a controller running at ``1 / period`` issues it;
    ``offset`` puts the motion on a gravity bias (``m g l sin(offset)``), as a
    leg joint holds a load while it walks. The file is read at first use, so a
    missing file only affects this trajectory.
    """

    def __init__(
        self,
        filename: str,
        offset: float = 0.5,
        lead_in: float = 1.0,
        period: float = 0.02,
    ):
        self.path = STREAMS_DIR / filename
        self.offset = offset
        self.lead_in = lead_in
        self.period = period
        self._goals = None

    def _load(self) -> np.ndarray:
        if self._goals is None:
            rows = [
                line.split(",")
                for line in self.path.read_text().splitlines()
                if line and line[0] != "#"
            ]
            self._goals = np.array(
                [float(row[1]) for row in rows[1:]]
            )  # rows[0] is the header
        return self._goals

    @property
    def duration(self) -> float:
        return self.lead_in + len(self._load()) * self.period

    def __call__(self, t: float) -> tuple[float, bool]:
        goals = self._load()
        start = self.offset + goals[0]
        if t < self.lead_in:
            return cubic_interpolate(
                [[0.0, 0.0, 0.0], [self.lead_in, start, 0.0]], t
            ), True
        k = min(int((t - self.lead_in) / self.period), len(goals) - 1)
        return self.offset + goals[k], True


STREAMS = {
    "gait_hip": TargetStream("gait_hip.csv"),
    "gait_knee": TargetStream("gait_knee.csv"),
}

trajectories.update(SMALL_SIGNAL)
trajectories.update(STREAMS)
