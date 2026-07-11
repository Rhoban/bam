# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""BAM actuator models for mjlab (MuJoCo Warp).

Wraps any BAM Model (m1–m6) into mjlab's actuator framework. All friction
computations are fully vectorized over the (num_envs, num_joints) batch
dimension using PyTorch.

Usage example — bundled motor::

    from bam.mjlab import make_bam_actuator_cfg

    actuator_cfg = make_bam_actuator_cfg(
        motor_name="xl330",
        model="m6",
        target_names_expr=(r".*",),
    )

Usage example — custom JSON path::

    actuator_cfg = make_bam_actuator_cfg(
        json_path="params/xl330/m6.json",
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

from mjlab.actuator.actuator import Actuator, ActuatorCfg, ActuatorCmd, CommandField
from mjlab.utils.spec import create_motor_actuator

from .actuator import VoltageControlledActuator
from .model import Model, load_model, _resolve_json_path

if TYPE_CHECKING:
    from mjlab.entity import Entity


# MuJoCo constraint-type id for a per-DOF friction constraint. In MuJoCo Warp the
# ``efc.id`` of such a constraint is the DOF index and its Jacobian is a unit row
# on that DOF, so it contributes exactly ``efc.force`` to ``qfrc_constraint``.
_FRICTION_DOF_CONSTRAINT = int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)


@dataclass(kw_only=True)
class BamActuatorCfg(ActuatorCfg):
    """Configuration for a BAM actuator compatible with mjlab.

    Specify the model with **one** of two mutually exclusive approaches:

    * **Bundled motor**: set ``motor_name`` (e.g. "xl330") and ``model``
      (e.g. "m6"). The path is resolved automatically from the ``params/``
      directory bundled with the library.
    * **Custom JSON**: set ``json_path`` to a BAM params JSON file produced by
      ``bam.fit``.

    :param motor_name: Name of the bundled motor. Currently supported: "xl330", "xl320", "mx106", "mx64", "erob80:50", and "erob80:100". Mutually exclusive with ``json_path``.
    :param model: Model variant to use with ``motor_name``, one of "m1"–"m6". Mutually exclusive with ``json_path``.
    :param json_path: Path to a custom BAM params JSON file produced by ``bam.fit``. Mutually exclusive with ``motor_name`` and ``model``.
    :param target_names_expr: Tuple of regex patterns to match actuated joint names.
    :param vin: Supply voltage override [V]. ``None`` → uses the value in the JSON.
    :param kp_fw: Firmware P-gain override. ``None`` → uses the value in the JSON.
    :param vin_range: If set, a per-env battery voltage is sampled uniformly from this
        range at startup and held constant across resets. Takes precedence over ``vin``.
    :param vin_drop_gain_range: If set, a per-env internal-resistance gain [V/Nm] is sampled uniformly
        from this range at startup. Models the voltage drop V_drop = gain * Σ|τ| due to
        battery + cable resistance. Gain should be approximately resistance / Kt.
        Held constant across resets.
    :param vin_min: Hard lower bound on the effective supply voltage [V] after applying the
        voltage drop. Ensures ``vin`` never falls below this value regardless of the load.
        ``None`` → no lower bound.
    :param max_current: Firmware current limit [A]. If set, the motor current is clipped to
        ``[-max_current, max_current]`` (equivalently the motor torque is clipped to
        ``±max_current * kt``), reproducing the firmware-side current saturation of the
        motor. ``None`` → no current clipping.
    :param delay_min_lag: Minimum command delay in simulation steps. Models the latency
        between the policy output and the motor response. ``0`` → no delay.
    :param delay_max_lag: Maximum command delay in simulation steps. Set greater than
        ``delay_min_lag`` to randomize the delay across environments.
    :param delay_hold_prob: Probability of keeping the same lag value at each step.
        ``0.0`` → lag is resampled every ``delay_update_period`` steps.
    :param delay_update_period: Number of steps between lag updates. ``0`` → updated
        every step.
    :param delay_per_env_phase: Whether each environment starts with an independent
        delay phase offset. ``True`` → environments are not synchronized.
    """

    motor_name: str | None = None
    model: str | None = None
    json_path: str | None = None
    vin: float | None = None
    kp_fw: float | None = None
    vin_range: tuple[float, float] | None = None
    vin_drop_gain_range: tuple[float, float] | None = None
    vin_min: float | None = None
    max_current: float | None = None

    def __post_init__(self) -> None:
        if self.json_path is not None and (
            self.motor_name is not None or self.model is not None
        ):
            raise ValueError(
                "Specify either json_path OR (motor_name + model), not both."
            )
        object.__setattr__(
            self,
            "_resolved_json_path",
            _resolve_json_path(self.json_path, self.motor_name, self.model),
        )

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

    Rather than injecting a passive friction torque into the returned motor
    torque, the friction budget is written into MuJoCo's native
    ``dof_frictionloss`` (dry friction) and ``dof_damping`` (viscous) each step,
    exactly like :class:`bam.mujoco.MujocoController`. MuJoCo's constraint solver
    then applies the static-friction clipping (BAM Algorithm 1) itself.

    .. important::
        Because every environment carries a different friction budget, the
        ``dof_frictionloss`` and ``dof_damping`` model fields must be expanded
        per world *before* stepping. After building the environment, call::

            env.sim.expand_model_fields(("dof_frictionloss", "dof_damping"))

        (or add a ``randomize_field`` event for those fields). Otherwise the
        fields alias a single shared buffer and per-env writes are invalid.

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
        self._bam_model: Model = load_model(cfg._resolved_json_path)

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
        self._device: str = "cpu"
        self._dof_ids: torch.Tensor | None = None

        self.vin_tensor: torch.Tensor | None = None
        self.vin_drop_gain: torch.Tensor | None = None
        self._prev_motor_torque: torch.Tensor | None = None

        self.kp_scale: torch.Tensor | None = None
        self.kd_scale: torch.Tensor | None = None
        self.default_kp_scale: torch.Tensor | None = None
        self.default_kd_scale: torch.Tensor | None = None

        self._num_envs: int = 0
        self._friction_fields_checked: bool = False

    # ─────────────────────────────────────────────────────────────────────────
    # mjlab interface
    # ─────────────────────────────────────────────────────────────────────────

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        """Convert position actuators to motor mode and zero MuJoCo friction.

        The ``frictionloss`` and ``damping`` are zeroed here only as an initial
        value; :meth:`compute` rewrites them every step with the BAM friction
        budget so MuJoCo's solver applies the friction natively.
        """
        bam = self._bam_model
        act = bam.actuator
        kt = bam.kt.value
        R = bam.R.value
        armature = act.get_extra_inertia()
        # Use upper bound of vin_range for force_limit so MuJoCo's forcerange
        # is always a safe ceiling regardless of per-env voltage.
        vin_for_limit = (
            max(self.cfg.vin_range) if self.cfg.vin_range is not None else act.vin
        )
        force_limit = vin_for_limit * kt / R

        target_set = set(target_names)
        converted: set[str] = set()

        for mjact in spec.actuators:
            tgt = mjact.target
            tgt_name = tgt.name if hasattr(tgt, "name") else (str(tgt) if tgt else None)
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
        self._device = device

        # Map local target indices → global joint indices → DOF addresses
        jnt_dofadr = mj_model.jnt_dofadr
        entity_joint_ids = self.entity.indexing.joint_ids
        dof_ids = [
            jnt_dofadr[entity_joint_ids[tid].item()] for tid in self._target_ids_list
        ]
        self._dof_ids = torch.tensor(dof_ids, dtype=torch.long, device=device)

        num_envs = data.nworld
        self._num_envs = num_envs
        self._friction_fields_checked = False
        num_joints = len(self._dof_ids)
        self.kp_scale = torch.ones(num_envs, 1, dtype=torch.float32, device=device)
        self.kd_scale = torch.ones(num_envs, 1, dtype=torch.float32, device=device)
        self.default_kp_scale = self.kp_scale.clone()
        self.default_kd_scale = self.kd_scale.clone()

        bam = self._bam_model
        act = bam.actuator

        # vin_tensor: (N, 1) — per-env battery voltage, constant across resets
        if self.cfg.vin_range is not None:
            self.vin_tensor = torch.empty(
                num_envs, 1, dtype=torch.float32, device=device
            ).uniform_(*self.cfg.vin_range)
        else:
            self.vin_tensor = torch.full(
                (num_envs, 1), act.vin, dtype=torch.float32, device=device
            )

        # vin_drop_gain: (N, 1) — per-env resistance gain [V/Nm], constant across resets
        if self.cfg.vin_drop_gain_range is not None:
            self.vin_drop_gain = torch.empty(
                num_envs, 1, dtype=torch.float32, device=device
            ).uniform_(*self.cfg.vin_drop_gain_range)
        else:
            self.vin_drop_gain = None

        # Previous motor torques for the voltage-drop computation (lagged 1 step)
        self._prev_motor_torque = torch.zeros(
            num_envs, num_joints, dtype=torch.float32, device=device
        )

        vin_repr = (
            f"range={self.cfg.vin_range}"
            if self.cfg.vin_range is not None
            else f"{act.vin:.1f}V"
        )
        drop_repr = (
            f"drop_gain_range={self.cfg.vin_drop_gain_range}"
            if self.cfg.vin_drop_gain_range is not None
            else "no drop"
        )
        vin_for_limit = (
            max(self.cfg.vin_range) if self.cfg.vin_range is not None else act.vin
        )
        force_limit = vin_for_limit * bam.kt.value / bam.R.value
        print(
            f"[BamActuator] model={bam.name!r} "
            f"joints={num_joints} "
            f"kt={bam.kt.value:.4f} R={bam.R.value:.4f} "
            f"vin={vin_repr} {drop_repr} force_limit=±{force_limit:.2f}Nm "
            f"friction_base={bam.friction_base.value:.4f} "
            f"friction_viscous={bam.friction_viscous.value:.4f} "
            f"envs={num_envs} device={device}"
        )

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        super().reset(env_ids)
        # vin_tensor and vin_drop_gain are startup-randomized: do NOT re-sample on reset.
        # Reset previous motor torques so the voltage-drop model starts clean.
        if self._prev_motor_torque is not None:
            if env_ids is None:
                self._prev_motor_torque.zero_()
            else:
                self._prev_motor_torque[env_ids] = 0.0

    @property
    def command_field(self) -> CommandField:
        return "position"

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
                    frictionloss + bam.load_friction_base.value * gearbox_torque
                )

                if bam.stribeck:
                    frictionloss = (
                        frictionloss
                        + stribeck_coeff
                        * bam.load_friction_stribeck.value
                        * gearbox_torque
                    )

        return frictionloss

    def _write_frictions(self, frictionloss: torch.Tensor, damping: float) -> None:
        """Write the friction budget into MuJoCo's per-DOF friction fields.

        ``frictionloss`` (shape ``(N, J)``) and ``damping`` (scalar viscous
        coefficient) are written into ``dof_frictionloss`` and ``dof_damping``
        of the controlled DOFs, for every environment. The fields are accessed
        fresh each call so that a later :meth:`~mjlab.sim.Sim.expand_model_fields`
        (which reallocates the arrays and clears the bridge cache) is picked up.
        """
        assert self._mjwarp_model is not None and self._dof_ids is not None

        fl_field = self._mjwarp_model.dof_frictionloss
        damping_field = self._mjwarp_model.dof_damping

        if not self._friction_fields_checked:
            # Per-env writes require truly per-world storage. A non-expanded
            # field aliases a single (1, nv) buffer (stride 0 on the world axis),
            # so writing distinct per-env values would be invalid.
            if self._num_envs > 1 and fl_field.stride(0) == 0:
                raise RuntimeError(
                    "BamActuator writes per-environment dof_frictionloss/"
                    "dof_damping, but these model fields are not expanded per "
                    "world. After building the environment, call "
                    "sim.expand_model_fields(('dof_frictionloss', "
                    "'dof_damping')) (or add a randomize_field event for them)."
                )
            self._friction_fields_checked = True

        fl_field[:, self._dof_ids] = frictionloss
        damping_field[:, self._dof_ids] = damping

    def _as_tensor(self, x) -> torch.Tensor:
        """Normalize a mjwarp/bridge field to a real ``torch.Tensor``.

        Handles mjlab's ``TorchArray`` bridge wrapper (indexing it returns the
        underlying tensor), plain tensors, and numpy fallbacks.
        """
        if isinstance(x, torch.Tensor):
            return x
        if hasattr(x, "_tensor"):  # mjlab TorchArray bridge wrapper
            return x[...]
        return torch.as_tensor(x, device=self._device)

    def _dof_friction_force(self, nv: int) -> torch.Tensor:
        """Per-DOF force produced by our own ``dof_frictionloss`` constraints.

        Scans the active constraint rows (``efc``) of the previous solve, keeps
        only the DOF-friction constraints, and scatters their constraint-space
        force onto the DOF they act on. Returns shape ``(N, nv)``.

        This is the MuJoCo-Warp equivalent of the ``efc``-scan done in
        :class:`bam.mujoco.MujocoController`, used to strip the friction
        contribution out of ``qfrc_constraint``.
        """
        assert self._data is not None
        efc = self._data.efc
        efc_type = self._as_tensor(efc.type)  # (N, njmax) int
        efc_id = self._as_tensor(efc.id)  # (N, njmax) int
        efc_force = self._as_tensor(efc.force)  # (N, njmax) float
        nefc = self._as_tensor(self._data.nefc)  # (N,) int

        n_worlds, n_max = efc_type.shape
        # Only the first nefc[w] rows of each world are valid; the rest is stale.
        valid = torch.arange(n_max, device=efc_type.device).unsqueeze(
            0
        ) < nefc.unsqueeze(1)
        is_fric = valid & (efc_type == _FRICTION_DOF_CONSTRAINT)  # (N, njmax)

        contrib = torch.where(is_fric, efc_force, torch.zeros_like(efc_force))
        # Masked-out rows scatter a zero contribution into DOF 0 (harmless).
        idx = torch.where(is_fric, efc_id, torch.zeros_like(efc_id)).long()

        qfrc_friction = torch.zeros(
            n_worlds, nv, dtype=efc_force.dtype, device=efc_force.device
        )
        qfrc_friction.scatter_add_(1, idx, contrib)
        return qfrc_friction

    # ─────────────────────────────────────────────────────────────────────────
    # Main compute — shape (num_envs, num_joints) throughout
    # ─────────────────────────────────────────────────────────────────────────

    def compute(self, cmd: ActuatorCmd) -> torch.Tensor:
        """Compute output torques for all environments — shape ``(N, J)``."""
        bam = self._bam_model
        act = bam.actuator

        # Read scalar params (cheap attribute reads, no tensor overhead)
        # vin_tensor is a (N,1) tensor initialized in initialize(); act.vin is no longer used directly.
        assert self.vin_tensor is not None
        vin = self.vin_tensor  # (N, 1)
        if self.vin_drop_gain is not None and self._prev_motor_torque is not None:
            load = self._prev_motor_torque.abs().sum(dim=-1, keepdim=True)  # (N, 1)
            vin = vin - self.vin_drop_gain * load  # (N, 1), broadcast safe
            if self.cfg.vin_min is not None:
                vin = torch.clamp(vin, min=self.cfg.vin_min)
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
        motor_torque = kt * voltage / R - (kt**2) * vel * self.kd_scale / R

        # Firmware current clipping: I = motor_torque / kt is capped at
        # ±max_current, i.e. the motor torque is clipped to ±max_current * kt.
        if self.cfg.max_current is not None:
            torque_limit = self.cfg.max_current * kt
            motor_torque = torch.clamp(motor_torque, -torque_limit, torque_limit)

        # ── 3. External torque (gravity + Coriolis + constraints) ─────────────
        # The external load on the gearbox is the gravity/Coriolis bias plus the
        # constraint forces (contacts, joint limits, …), but NOT the DOF-friction
        # constraint force we injected via dof_frictionloss on the previous solve
        # — otherwise the load-dependent friction terms would feed back on
        # themselves. This mirrors bam.mujoco.MujocoController.
        qfrc_bias = self._as_tensor(self._data.qfrc_bias)  # (N, nv)
        qfrc_constraint = self._as_tensor(self._data.qfrc_constraint)  # (N, nv)
        qfrc_actuator = self._as_tensor(self._data.qfrc_actuator)  # (N, nv)
        nv = qfrc_bias.shape[-1]
        qfrc_friction = self._dof_friction_force(nv)  # (N, nv)
        external_torque = (
            -qfrc_bias[:, self._dof_ids]
            + qfrc_constraint[:, self._dof_ids]
            - qfrc_friction[:, self._dof_ids]
        )  # (N, J)
        # Actuator torque applied on the PREVIOUS solve. The BAM friction budget
        # uses this (not the freshly-computed motor_torque) as the motor-side load,
        # matching bam.mujoco.MujocoController which reads data.qfrc_actuator.
        prev_actuator_torque = qfrc_actuator[:, self._dof_ids]  # (N, J)

        # ── 4. Stribeck coefficient ───────────────────────────────────────────
        # (N, J); zero tensor when model has no stribeck (unused in budget)
        abs_vel = torch.abs(vel)
        if bam.stribeck:
            dtheta_stribeck = bam.dtheta_stribeck.value
            alpha = bam.alpha.value
            stribeck_coeff = torch.exp(-torch.pow(abs_vel / dtheta_stribeck, alpha))
        else:
            stribeck_coeff = torch.zeros_like(vel)

        # ── 5. Friction budget → MuJoCo dof_frictionloss / dof_damping ────────
        # Velocity-independent (Coulomb + Stribeck + load) part → frictionloss.
        # Viscous part → damping. MuJoCo's constraint solver then performs the
        # static-friction clipping (BAM Algorithm 1) itself, so we do NOT add a
        # passive friction torque to the returned motor torque.
        frictionloss = self._compute_friction_budget(
            prev_actuator_torque, external_torque, stribeck_coeff
        )  # (N, J)
        self._write_frictions(frictionloss, friction_viscous)

        # Store motor torque for next step's voltage-drop computation
        if self._prev_motor_torque is not None:
            self._prev_motor_torque = motor_torque.detach()

        return motor_torque
