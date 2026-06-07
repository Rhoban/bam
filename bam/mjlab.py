# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""BAM actuator models for mjlab (MuJoCo Warp).

Wraps any BAM Model (m1–m6) into mjlab's actuator framework. All friction
computations are fully vectorized over the (num_envs, num_joints) batch
dimension using PyTorch.

Usage example::

    from bam.mjlab import make_bam_actuator_cfg

    actuator_cfg = make_bam_actuator_cfg(
        "params/xl330/m6.json",
        target_names_expr=(r".*",),
    )
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import mujoco
import mujoco_warp as mjwarp
import torch

from mjlab.actuator.actuator import Actuator, ActuatorCfg, ActuatorCmd
from mjlab.utils.spec import create_motor_actuator

from .actuator import VoltageControlledActuator
from .model import Model, load_model

if TYPE_CHECKING:
    from mjlab.entity import Entity


@dataclass(kw_only=True)
class BamActuatorCfg(ActuatorCfg):
    """Configuration for a BAM actuator compatible with mjlab.

    The friction model (m1–m6) is inferred from the flags stored in the JSON
    file (``load_dependent``, ``directional``, ``stribeck``, ``quadratic``).
    Only :class:`~bam.actuator.VoltageControlledActuator` is supported.

    Using a JSON path (instead of a ``Model`` object) makes this config fully
    serializable by tyro and safely copyable across processes.

    Args:
        json_path: Path to a BAM params JSON file (output of ``bam.fit``).
        target_names_expr: Tuple of regex patterns to match actuated joint names.
        vin: Supply voltage override [V]. ``None`` → uses the value in the JSON.
        kp_fw: Firmware P-gain override. ``None`` → uses the value in the JSON.
    """

    json_path: str
    vin: float | None = None
    kp_fw: float | None = None

    def build(
        self,
        entity: "Entity",
        target_ids: list[int],
        target_names: list[str],
    ) -> "BamActuator":
        return BamActuator(self, entity, target_ids, target_names)


class BamActuator(Actuator):
    """BAM actuator for mjlab — fully vectorized over all parallel environments.

    Implements the BAM torque pipeline:

    1. **Voltage control law** — firmware P-controller (position error → duty
       cycle → voltage).
    2. **DC motor torque** — back-EMF equation (voltage → torque).
    3. **Friction budget** — BAM m1–m6 friction model (Coulomb, Stribeck,
       load-dependent, directional, quadratic).
    4. **Static friction clipping** — BAM Algorithm 1 (prevents overshoot when
       the joint is nearly stopped).

    Per-environment gain scaling is supported via :meth:`set_gains`.
    """

    cfg: BamActuatorCfg

    def __init__(
        self,
        cfg: BamActuatorCfg,
        entity: "Entity",
        target_ids: list[int],
        target_names: list[str],
    ) -> None:
        super().__init__(cfg, entity, target_ids, target_names)

        # Load the BAM model from JSON (one instance per actuator)
        self._bam_model: Model = load_model(cfg.json_path)

        if cfg.vin is not None:
            self._bam_model.actuator.vin = cfg.vin
        if cfg.kp_fw is not None:
            self._bam_model.actuator.kp = cfg.kp_fw

        if not isinstance(self._bam_model.actuator, VoltageControlledActuator):
            raise NotImplementedError(
                f"BamActuator only supports VoltageControlledActuator, "
                f"got {type(self._bam_model.actuator).__name__}"
            )

        self._mjwarp_model: mjwarp.Model | None = None
        self._data: mjwarp.Data | None = None
        self._dt: float = 0.0
        self._device: str = "cpu"
        self._dof_ids: torch.Tensor | None = None

        self.kp_scale: torch.Tensor | None = None
        self.kd_scale: torch.Tensor | None = None
        self.default_kp_scale: torch.Tensor | None = None
        self.default_kd_scale: torch.Tensor | None = None

    # ─────────────────────────────────────────────────────────────────────────
    # mjlab interface
    # ─────────────────────────────────────────────────────────────────────────

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        """Convert position actuators to motor mode and zero MuJoCo friction.

        We handle all friction ourselves inside :meth:`compute`, so MuJoCo's
        built-in ``frictionloss`` and ``damping`` are zeroed out here.
        """
        bam = self._bam_model
        act = bam.actuator
        vin = act.vin  # already overridden in __init__ if cfg.vin was set
        kt = bam.kt.value
        R = bam.R.value
        armature = act.get_extra_inertia()
        force_limit = vin * kt / R

        target_set = set(target_names)
        converted: set[str] = set()

        for mjact in spec.actuators:
            tgt = mjact.target
            tgt_name = (
                tgt.name
                if hasattr(tgt, "name")
                else (str(tgt) if tgt else None)
            )
            if tgt_name in target_set:
                mjact.set_to_motor()
                mjact.forcelimited = True
                mjact.forcerange = (-force_limit, force_limit)
                mjact.gear = [1.0, 0, 0, 0, 0, 0]
                for joint in spec.joints:
                    if joint.name == tgt_name:
                        joint.armature = float(armature)
                        joint.damping = np.zeros((3, 1))
                        joint.frictionloss = 0.0
                        break
                self._mjs_actuators.append(mjact)
                converted.add(tgt_name)

        for target_name in target_names:
            if target_name not in converted:
                mjact = create_motor_actuator(
                    spec,
                    target_name,
                    effort_limit=force_limit,
                    armature=armature,
                    frictionloss=0.0,
                    transmission_type=self.cfg.transmission_type,
                )
                self._mjs_actuators.append(mjact)
                for joint in spec.joints:
                    if joint.name == target_name:
                        joint.damping = np.zeros((3, 1))
                        joint.frictionloss = 0.0
                        break

    def initialize(
        self,
        mj_model: mujoco.MjModel,
        model: mjwarp.Model,
        data: mjwarp.Data,
        device: str,
    ) -> None:
        super().initialize(mj_model, model, data, device)
        self._mjwarp_model = model
        self._data = data
        self._dt = mj_model.opt.timestep
        self._device = device

        # Map local target indices → global joint indices → DOF addresses
        jnt_dofadr = mj_model.jnt_dofadr
        entity_joint_ids = self.entity.indexing.joint_ids
        dof_ids = [
            jnt_dofadr[entity_joint_ids[tid].item()]
            for tid in self._target_ids_list
        ]
        self._dof_ids = torch.tensor(dof_ids, dtype=torch.long, device=device)

        num_envs = data.nworld
        self.kp_scale = torch.ones(num_envs, 1, dtype=torch.float32, device=device)
        self.kd_scale = torch.ones(num_envs, 1, dtype=torch.float32, device=device)
        self.default_kp_scale = self.kp_scale.clone()
        self.default_kd_scale = self.kd_scale.clone()

        bam = self._bam_model
        act = bam.actuator
        vin = act.vin
        force_limit = vin * bam.kt.value / bam.R.value
        print(
            f"[BamActuator] model={bam.name!r} "
            f"joints={len(self._dof_ids)} "
            f"kt={bam.kt.value:.4f} R={bam.R.value:.4f} "
            f"vin={vin:.1f}V force_limit=±{force_limit:.2f}Nm "
            f"friction_base={bam.friction_base.value:.4f} "
            f"friction_viscous={bam.friction_viscous.value:.4f} "
            f"envs={num_envs} device={device}"
        )

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        super().reset(env_ids)

    # ─────────────────────────────────────────────────────────────────────────
    # Gain scaling (for domain randomization)
    # ─────────────────────────────────────────────────────────────────────────

    def set_gains(
        self,
        env_ids: torch.Tensor | slice,
        kp_scale: torch.Tensor | None = None,
        kd_scale: torch.Tensor | None = None,
    ) -> None:
        """Scale firmware gains for a subset of environments."""
        if kp_scale is not None:
            assert self.kp_scale is not None
            self.kp_scale[env_ids] = kp_scale
        if kd_scale is not None:
            assert self.kd_scale is not None
            self.kd_scale[env_ids] = kd_scale

    def reset_gains(self, env_ids: torch.Tensor | slice) -> None:
        """Restore default gains for a subset of environments."""
        assert self.kp_scale is not None and self.default_kp_scale is not None
        assert self.kd_scale is not None and self.default_kd_scale is not None
        self.kp_scale[env_ids] = self.default_kp_scale[env_ids]
        self.kd_scale[env_ids] = self.default_kd_scale[env_ids]

    # ─────────────────────────────────────────────────────────────────────────
    # BAM friction budget (m1–m6 unified, vectorized)
    # ─────────────────────────────────────────────────────────────────────────

    def _compute_friction_budget(
        self,
        motor_torque: torch.Tensor,
        external_torque: torch.Tensor,
        stribeck_coeff: torch.Tensor,
    ) -> torch.Tensor:
        """Velocity-independent friction budget — shape ``(N, J)``.

        Covers all BAM models m1–m6 by reading flags from the stored Model:

        * **m1**: Coulomb only (``friction_base``)
        * **m2**: + Stribeck (``stribeck=True``)
        * **m3**: + non-directional load friction (``load_dependent=True``)
        * **m4**: m3 + Stribeck load friction
        * **m5**: directional load friction (``directional=True``)
        * **m6**: m5 + quadratic Stribeck load term (``quadratic=True``)
        """
        bam = self._bam_model
        frictionloss = torch.full_like(motor_torque, bam.friction_base.value)

        if bam.stribeck:
            frictionloss = frictionloss + stribeck_coeff * bam.friction_stribeck.value

        if bam.load_dependent:
            if bam.directional:
                # m5/m6 — directional gearbox torque
                gearbox_torque = torch.abs(
                    external_torque * bam.load_friction_external.value
                    - motor_torque * bam.load_friction_motor.value
                )
                frictionloss = frictionloss + gearbox_torque

                if bam.stribeck:
                    gearbox_torque_stribeck = torch.abs(
                        external_torque * bam.load_friction_external_stribeck.value
                        - motor_torque * bam.load_friction_motor_stribeck.value
                    )
                    frictionloss = (
                        frictionloss + stribeck_coeff * gearbox_torque_stribeck
                    )

                    if bam.quadratic:
                        # m6 — quadratic term; directional: motor-side vs external-side
                        abs_ext = torch.abs(external_torque)
                        abs_mot = torch.abs(motor_torque)
                        drive_mask = (abs_mot > abs_ext).to(motor_torque.dtype)
                        backdrive_mask = 1.0 - drive_mask
                        quad_term = (
                            drive_mask
                            * bam.load_friction_external_quad.value
                            * abs_ext**2
                            + backdrive_mask
                            * bam.load_friction_motor_quad.value
                            * abs_mot**2
                        )
                        frictionloss = frictionloss + stribeck_coeff * quad_term
            else:
                # m3/m4 — non-directional gearbox torque
                gearbox_torque = torch.abs(external_torque - motor_torque)
                frictionloss = (
                    frictionloss
                    + bam.load_friction_base.value * gearbox_torque
                )

                if bam.stribeck:
                    frictionloss = (
                        frictionloss
                        + stribeck_coeff
                        * bam.load_friction_stribeck.value
                        * gearbox_torque
                    )

        return frictionloss

    # ─────────────────────────────────────────────────────────────────────────
    # Main compute — shape (num_envs, num_joints) throughout
    # ─────────────────────────────────────────────────────────────────────────

    def compute(self, cmd: ActuatorCmd) -> torch.Tensor:
        """Compute output torques for all environments — shape ``(N, J)``."""
        bam = self._bam_model
        act = bam.actuator

        # Read scalar params (cheap attribute reads, no tensor overhead)
        # vin / kp_fw overrides were already applied to act in __init__
        vin = act.vin
        kp_fw = act.kp
        kt = bam.kt.value
        R = bam.R.value
        error_gain = act.error_gain
        max_pwm = act.max_pwm
        friction_viscous = bam.friction_viscous.value

        assert self.kp_scale is not None and self.kd_scale is not None
        assert self._data is not None and self._dof_ids is not None
        assert self._mjwarp_model is not None

        # ── 1. Firmware voltage control law ──────────────────────────────────
        # (N, J)
        pos_error = cmd.position_target - cmd.pos
        duty_cycle = pos_error * kp_fw * self.kp_scale * error_gain
        duty_cycle = torch.clamp(duty_cycle, -max_pwm, max_pwm)
        voltage = vin * duty_cycle

        # ── 2. DC motor torque with back-EMF ─────────────────────────────────
        # (N, J)
        vel = cmd.vel
        motor_torque = (
            kt * voltage / R
            - (kt**2) * vel * self.kd_scale / R
        )

        # ── 3. External (gravity + Coriolis) torque ───────────────────────────
        # qfrc_bias shape: (N, ndof) — slice to (N, J)
        qfrc_bias_raw = self._data.qfrc_bias
        if not isinstance(qfrc_bias_raw, torch.Tensor):
            qfrc_bias_raw = torch.as_tensor(qfrc_bias_raw, device=self._device)
        external_torque = -qfrc_bias_raw[:, self._dof_ids]  # (N, J)

        # ── 4. Stribeck coefficient ───────────────────────────────────────────
        # (N, J); zero tensor when model has no stribeck (unused in budget)
        abs_vel = torch.abs(vel)
        if bam.stribeck:
            dtheta_stribeck = bam.dtheta_stribeck.value
            alpha = bam.alpha.value
            stribeck_coeff = torch.exp(
                -torch.pow(abs_vel / dtheta_stribeck, alpha)
            )
        else:
            stribeck_coeff = torch.zeros_like(vel)

        # ── 5. Friction budget ────────────────────────────────────────────────
        # (N, J)  velocity-independent part
        frictionloss = self._compute_friction_budget(
            motor_torque, external_torque, stribeck_coeff
        )
        # Add viscous friction to get total budget
        friction_budget = frictionloss + friction_viscous * abs_vel  # (N, J)

        # ── 6. Static friction clipping — BAM Algorithm 1 ────────────────────
        # Effective inertia from MuJoCo's dof_invweight0
        dof_invweight = self._mjwarp_model.dof_invweight0
        if not isinstance(dof_invweight, torch.Tensor):
            dof_invweight = torch.as_tensor(dof_invweight, device=self._device)
        if dof_invweight.ndim == 1:
            eff_inertia = 1.0 / dof_invweight[self._dof_ids].unsqueeze(0)  # (1, J)
        else:
            eff_inertia = 1.0 / dof_invweight[:, self._dof_ids]  # (N, J)

        # Torque needed to stop the joint in one timestep (absent friction):
        #   tau_stop = (I/dt)*vel + motor_torque + qfrc_bias
        #            = (I/dt)*vel + motor_torque - external_torque
        net_no_friction = motor_torque - external_torque  # (N, J)
        tau_stop = (eff_inertia / self._dt) * vel + net_no_friction  # (N, J)

        friction_magnitude = torch.minimum(torch.abs(tau_stop), friction_budget)
        friction_torque = -torch.sign(tau_stop) * friction_magnitude  # (N, J)

        return motor_torque + friction_torque  # (N, J)


# ─────────────────────────────────────────────────────────────────────────────
# Factory helper
# ─────────────────────────────────────────────────────────────────────────────


def make_bam_actuator_cfg(
    json_path: str | Path,
    target_names_expr: tuple[str, ...] = (r".*",),
    vin: float | None = None,
    kp_fw: float | None = None,
    delay_min_lag: int = 0,
    delay_max_lag: int = 0,
    delay_hold_prob: float = 0.0,
    delay_update_period: int = 0,
    delay_per_env_phase: bool = True,
) -> BamActuatorCfg:
    """Create a :class:`BamActuatorCfg` from a BAM params JSON file.

    Args:
        json_path: Path to a BAM params JSON file (output of ``bam.fit``).
        target_names_expr: Regex patterns to match actuated joint names.
        vin: Supply voltage override [V]. ``None`` → uses the value in the JSON.
        kp_fw: Firmware P-gain override. ``None`` → uses the value in the JSON.
        delay_min_lag: Minimum observation delay in simulation steps.
        delay_max_lag: Maximum observation delay in simulation steps.
        delay_hold_prob: Probability of holding the same lag value each step.
        delay_update_period: Number of steps between lag updates (0 = every step).
        delay_per_env_phase: Whether each env has an independent delay phase.

    Returns:
        A :class:`BamActuatorCfg` ready to pass as ``actuator_cfgs`` to an
        mjlab Entity.

    Example::

        from bam.mjlab import make_bam_actuator_cfg

        cfg = make_bam_actuator_cfg(
            "params/xl330/m6.json",
            target_names_expr=(r".*",),
            kp_fw=125,
            vin=8.0,
            delay_min_lag=0,
            delay_max_lag=3,
        )
    """
    return BamActuatorCfg(
        target_names_expr=target_names_expr,
        json_path=str(json_path),
        vin=vin,
        kp_fw=kp_fw,
        delay_min_lag=delay_min_lag,
        delay_max_lag=delay_max_lag,
        delay_hold_prob=delay_hold_prob,
        delay_update_period=delay_update_period,
        delay_per_env_phase=delay_per_env_phase,
    )
