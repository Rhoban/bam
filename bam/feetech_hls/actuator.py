# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

from __future__ import annotations
from typing import TYPE_CHECKING
import numpy as np

from bam.actuator import CurrentControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench

if TYPE_CHECKING:
    from bam.actuator import ArrayLike


def pd_current(q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, kp: float, damping: float) -> ArrayLike:
    """Unclamped PD current command [A], shared by the recorder and the model.

    In constant-current mode the position loop runs on the host, so the same law
    has to be executed twice: once on the servo bus
    (:mod:`bam.feetech_hls.record`) and once when the model replays the log
    (``bam.fit`` re-derives the control with ``simulate_control=True``). Keeping
    a single implementation here is what stops the two from drifting apart:
    :meth:`FeetechHLSActuator.compute_control` and the recorder both call it.

    :param q_target: Target joint angle(s) [rad].
    :param q: Current joint angle(s) [rad].
    :param dq: Current joint velocity(ies) [rad/s].
    :param kp: Proportional gain.
    :param damping: Damping gain, scaled by ``sqrt(kp)`` so that the loop stays
        critically damped when ``kp`` changes.
    :returns: Commanded phase current [A], before saturation.
    """
    # Target velocity is assumed to be 0
    return (q_target - q) * kp + damping * np.sqrt(kp) * (0.0 - dq)


class FeetechHLSActuator(CurrentControlledActuator):
    """
    Feetech HLS magnetic-encoder servo operated in constant-current mode.

    In constant-current mode the servo runs no position loop of its own: the
    host closes the loop and streams target currents (see
    :mod:`bam.feetech_hls.record`). The law therefore lives here and must stay
    identical to the recorder's, the same convention as :class:`bam.erob.actuator.ErobActuator`.

    Torque is proportional to the commanded phase current:

    .. math::

        \\tau = k_t \\cdot I \\cdot \\text{torque\\_enable}
    """

    def __init__(
        self,
        testbench_class: Testbench,
        vin: float = 12.0,
        kp: float = 3.0,
        damping: float = 0.08,
        max_amps: float = 0.36,
    ):
        super().__init__(testbench_class, vin=vin, kp=kp)
        self.damping = damping
        self.max_amps = max_amps

    def load_log(self, log: dict):
        super().load_log(log)
        if "damping" in log:
            self.damping = log["damping"]
        if "max_amps" in log:
            self.max_amps = log["max_amps"]

    def control_unit(self) -> str:
        return "amps"

    def initialize(self):
        super().initialize()

        # Torque constant [N.m/A], physical: with the measured current scale the
        # 11.0 kg.cm peak falls at 0.98 A -> 0.97 N.m/A, against 0.93 implied by
        # the datasheet stall point (14.2 kg.cm / 1.5 A) and ~1.0 by its no-load
        # speed. Bounds bracket all three.
        self.model.kt = Parameter(0.97, 0.5, 1.6)

        # Motor resistance [Ohm] — measured 7.56 at 100% duty / 4.8 V and 7.06 at
        # 12 V with a 22.7% duty cycle; the datasheet's 1.5 A stall at 12 V
        # implies 8.0. Held fixed rather than fitted: in constant-current mode the
        # control law is a PD on current and the torque is kt*I, so R never enters
        # the simulation and the data cannot constrain it - left free it just
        # wandered between fits (7.4 and 4.1 ohm for the same rig). It is still a
        # Parameter because bam.to_mujoco derives the exported model from it.
        self.model.R = Parameter(7.5, 4.0, 11.0, optimize=False)

        # Apparent rotor inertia [kg m^2]. Both fits so far pushed this to the top
        # of a 2e-3 bound, which is plausible rather than suspicious: the reflected
        # rotor inertia scales with the gear ratio squared, and a ~1:300 gearbox
        # turns a ~1e-8 kg m^2 rotor into a few 1e-3 at the output. The bound is
        # therefore opened up so the fit is not clipped.
        self.model.armature = Parameter(0.001, 0.00001, 0.006)

        # Joint angle zero offset [rad]
        self.model.q_offset = Parameter(0.0, -0.5, 0.5)

        # Friction bounds scaled to a 14.2 kg.cm-class micro servo. The Model
        # defaults (0.2 / 0.5 / 1.0 Nm) target much larger actuators: its
        # operating torque is only ~0.5 Nm, so a 0.2 Nm Coulomb bound would let
        # the optimizer bury the motor torque in friction.
        self.model.max_friction_base = 0.08
        self.model.max_load_friction = 0.15
        self.model.max_viscous_friction = 0.2

        # Physical current ceiling of this servo (measured 1.37 A at 12 V with the
        # bridge fully on), not the recorder's clamp: bam.to_mujoco derives the
        # MuJoCo forcerange from it, which wants the actuator's real limit.
        # Held fixed, and deliberately *not* synced from the log: bam.fit
        # serializes a fresh model that never saw a log, so anything load_log
        # writes would be lost in the saved params anyway.
        self.model.current_limit = Parameter(1.37, 0.5, 2.0, optimize=False)

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> ArrayLike | None:
        """PD on current, the same law :mod:`bam.feetech_hls.record` streams.

        :param q_target: Target joint angle(s) [rad].
        :param q: Current joint angle(s) [rad].
        :param dq: Current joint velocity(ies) [rad/s].
        :param dt: Timestep [s] (unused here).
        :returns: Commanded phase current [A].
        """
        amps = pd_current(q_target, q, dq, self.kp, self.damping)
        return self.backend.clamp(amps, -self.max_amps, self.max_amps)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def compute_torque(
        self,
        control: ArrayLike | None,
        torque_enable: bool,
        q: ArrayLike,
        dq: ArrayLike,
    ) -> ArrayLike:
        """Motor torque from the commanded phase current: :math:`\\tau = k_t I`.

        :param control: Commanded phase current [A].
        :param torque_enable: If ``False``, returns zero torque.
        :param q: Current joint angle(s) [rad] (unused here).
        :param dq: Current joint velocity(ies) [rad/s] (unused here).
        :returns: Motor torque [Nm].
        """
        if control is None:
            return 0.0
        torque = self.model.kt.value * control
        return torque * torque_enable
