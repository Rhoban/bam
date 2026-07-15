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
from mjlab.scene import Scene, SceneCfg
from mjlab.sim import MujocoCfg, Simulation, SimulationCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.managers.event_manager import RecomputeLevel, requires_model_fields

from .actuator import TorchBackend, VoltageControlledActuator
from .model import Model, load_model, _resolve_json_path
from .testbench_mujoco import Pendulum

if TYPE_CHECKING:
    from mjlab.entity import Entity


@requires_model_fields("dof_frictionloss", "dof_damping")
def bam_init(env, env_ids=None) -> None:
    """Startup event that expands BamActuator's per-world friction fields.

    :class:`BamActuator` writes a per-environment friction budget into MuJoCo's
    ``dof_frictionloss`` and ``dof_damping`` model fields. Those fields must be
    expanded per world (otherwise they alias one shared buffer and per-env writes
    are invalid). Add this as a ``startup`` event and mjlab expands them for you::

        from bam.mjlab import bam_init
        from mjlab.managers.event_manager import EventTermCfg

        events["bam_init"] = EventTermCfg(func=bam_init, mode="startup")

    The body is intentionally a no-op: the ``@requires_model_fields`` decorator
    does the work by registering the fields in the EventManager, which the env
    then hands to ``sim.expand_model_fields()`` during setup.
    """
    del env, env_ids


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
    :param vin_drop_resistance_range: If set, a per-env battery + wire resistance [Ohm] is sampled
        uniformly from this range at startup. Models the voltage drop V_drop = R * I due to
        battery + cable resistance, where the current I [A] is estimated from the actuator
        torques as Σ|τ| / Kt. Held constant across resets.
    :param vin_min: Hard lower bound on the effective supply voltage [V] after applying the
        voltage drop. Ensures ``vin`` never falls below this value regardless of the load.
        ``None`` → no lower bound.
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
    :param stiff_frictionloss: When ``True`` (default), stiffen the joint-friction
        constraint (``solref_friction`` / ``solimp_friction``). MuJoCo Warp has no
        noslip solver, so the frictionloss constraint stays soft and a statically-held
        joint creeps; this is the GPU-side substitute. Uses MuJoCo's direct
        (timestep-independent) solref form so it works regardless of the sim ``dt``.
        Set ``False`` to keep MuJoCo's soft defaults.
    """

    motor_name: str | None = None
    model: str | None = None
    json_path: str | None = None
    vin: float | None = None
    kp_fw: float | None = None
    vin_range: tuple[float, float] | None = None
    vin_drop_resistance_range: tuple[float, float] | None = None
    vin_min: float | None = None
    stiff_frictionloss: bool = True

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
        per world *before* stepping. The simplest way is to add the
        :func:`bam_init` startup event::

            from bam.mjlab import bam_init
            events["bam_init"] = EventTermCfg(func=bam_init, mode="startup")

        (equivalently, call ``env.sim.expand_model_fields(("dof_frictionloss",
        "dof_damping"))`` after building the environment). Otherwise the fields
        alias a single shared buffer and per-env writes are invalid.

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
        self.vin_drop_resistance: torch.Tensor | None = None

        self.kp_scale: torch.Tensor | None = None
        self.kd_scale: torch.Tensor | None = None
        self.default_kp_scale: torch.Tensor | None = None
        self.default_kd_scale: torch.Tensor | None = None

        self._base_kp: float = 0.0
        self._dt: float = 0.0

        self._num_envs: int = 0
        self._friction_fields_checked: bool = False

    # ─────────────────────────────────────────────────────────────────────────
    # mjlab interface
    # ─────────────────────────────────────────────────────────────────────────

    # Stiff joint-friction constraint, in MuJoCo's direct (timestep-independent)
    # solref form: solref = (-stiffness, -damping), solimp with dmax→1. Applied
    # when cfg.stiff_frictionloss is True to counter the lack of a noslip solver
    # in MuJoCo Warp (matches a (2*dt, 1.0) timeconst solref at dt≈5 ms).
    _STIFF_SOLREF_FRICTION = (-5.0e4, -2.0e2)
    _STIFF_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)

    def _set_friction_stiffness(self, joint: "mujoco.MjsJoint") -> None:
        """Stiffen a joint's friction constraint if ``cfg.stiff_frictionloss``."""
        if self.cfg.stiff_frictionloss:
            joint.solref_friction = self._STIFF_SOLREF_FRICTION
            joint.solimp_friction = self._STIFF_SOLIMP_FRICTION

    def edit_spec(self, spec: mujoco.MjSpec, target_names: list[str]) -> None:
        """Convert position actuators to motor mode and zero MuJoCo friction.

        The ``frictionloss`` and ``damping`` are zeroed here only as an initial
        value; :meth:`compute` rewrites them every step with the BAM friction
        budget so MuJoCo's solver applies the friction natively. When
        ``cfg.stiff_frictionloss`` is set, the friction constraint's
        ``solref_friction`` / ``solimp_friction`` are stiffened here too (see
        :meth:`_set_friction_stiffness`).
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
                        self._set_friction_stiffness(joint)
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
                        self._set_friction_stiffness(joint)
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

        # Delegate the control law / torque equation to the BAM actuator, run on
        # the Torch backend so its clamps are vectorized over (num_envs, num_joints).
        act.backend = TorchBackend()
        # Base firmware gain and physics timestep, captured before compute() starts
        # overwriting act.kp / act.vin with per-env tensors each step.
        self._base_kp = float(act.kp)
        self._dt = float(mj_model.opt.timestep)

        # vin_tensor: (N, 1) — per-env battery voltage, constant across resets
        if self.cfg.vin_range is not None:
            self.vin_tensor = torch.empty(
                num_envs, 1, dtype=torch.float32, device=device
            ).uniform_(*self.cfg.vin_range)
        else:
            self.vin_tensor = torch.full(
                (num_envs, 1), act.vin, dtype=torch.float32, device=device
            )

        # vin_drop_resistance: (N, 1) — per-env battery + wire resistance [Ohm], constant across resets
        if self.cfg.vin_drop_resistance_range is not None:
            self.vin_drop_resistance = torch.empty(
                num_envs, 1, dtype=torch.float32, device=device
            ).uniform_(*self.cfg.vin_drop_resistance_range)
        else:
            self.vin_drop_resistance = None

        vin_repr = (
            f"range={self.cfg.vin_range}"
            if self.cfg.vin_range is not None
            else f"{act.vin:.1f}V"
        )
        drop_repr = (
            f"drop_resistance_range={self.cfg.vin_drop_resistance_range}"
            if self.cfg.vin_drop_resistance_range is not None
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
        # vin_tensor and vin_drop_resistance are startup-randomized: do NOT re-sample on reset.

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
            if self._num_envs > 1 and (fl_field.stride(0) == 0 or damping_field.stride(0) == 0):
                raise RuntimeError(
                    "BamActuator writes per-environment dof_frictionloss/"
                    "dof_damping, but these model fields are not expanded per "
                    "world. Add the bam_init startup event so mjlab expands them "
                    "automatically:\n"
                    "    from bam.mjlab import bam_init\n"
                    "    events['bam_init'] = EventTermCfg(func=bam_init, "
                    "mode='startup')\n"
                    "Alternatively, after building the environment, call "
                    "sim.expand_model_fields(('dof_frictionloss', "
                    "'dof_damping'))."
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
        """Compute output torques for all environments — shape ``(N, J)``.

        The firmware control law and the DC-motor torque equation are delegated
        to the underlying BAM actuator's :meth:`~bam.actuator.Actuator.compute_control`
        and :meth:`~bam.actuator.Actuator.compute_torque` (running on the Torch
        backend, so their clamps vectorize over ``(N, J)``). This backend only
        injects the mjlab-specific per-environment quantities around those calls:

        * ``vin`` (with the optional battery drop) → ``actuator.vin``
        * ``kp * kp_scale`` → ``actuator.kp``
        * ``kd_scale`` → applied to the back-EMF via a scaled velocity
        """
        bam = self._bam_model
        act = bam.actuator

        assert self.vin_tensor is not None
        assert self.kp_scale is not None and self.kd_scale is not None
        assert self._data is not None and self._dof_ids is not None
        assert self._mjwarp_model is not None

        # Actuator torque applied on the PREVIOUS solve (lagged one step). The BAM
        # friction budget uses this (not the freshly-computed motor_torque) as the
        # motor-side load, and the battery voltage-drop model reuses it as the
        # current-draw proxy. Mirrors bam.mujoco.MujocoController.
        prev_actuator_torque = self._as_tensor(self._data.qfrc_actuator)[
            :, self._dof_ids
        ]  # (N, J)

        # ── Per-env supply voltage (with optional battery drop) ──────────────
        # V_drop = R * I, with the current I estimated from the actuator torques
        # as Σ|τ| / kt across the controlled joints.
        vin = self.vin_tensor  # (N, 1)
        if self.vin_drop_resistance is not None:
            current = (
                prev_actuator_torque.abs().sum(dim=-1, keepdim=True) / bam.kt.value
            )  # (N, 1)
            vin = vin - self.vin_drop_resistance * current  # (N, 1), broadcast safe
            if self.cfg.vin_min is not None:
                vin = torch.clamp(vin, min=self.cfg.vin_min)

        friction_viscous = bam.friction_viscous.value
        vel = cmd.vel  # (N, J)

        # ── 1-2. Firmware control law + DC-motor torque (delegated) ──────────
        # Inject the per-env firmware parameters: compute_control multiplies by
        # actuator.kp and actuator.vin, so writing per-env tensors there yields
        # the vectorized firmware controller with no re-implementation.
        act.vin = vin  # (N, 1)
        act.kp = self._base_kp * self.kp_scale  # (N, 1)
        # kd_scale scales the electrical (back-EMF) damping only. Both
        # compute_control (current-limiter back-EMF) and compute_torque
        # (motor-torque back-EMF) use the velocity solely in that term, so
        # feeding them a scaled velocity applies kd_scale consistently. The
        # firmware current limit is modelled inside compute_control as a
        # duty-cycle constraint (torque_enable handled by the Simulator).
        scaled_vel = vel * self.kd_scale
        control = act.compute_control(
            cmd.position_target, cmd.pos, scaled_vel, self._dt
        )
        motor_torque = act.compute_torque(control, True, cmd.pos, scaled_vel)

        # ── 3. External torque (gravity + Coriolis + constraints) ─────────────
        # The external load on the gearbox is the gravity/Coriolis bias plus the
        # constraint forces (contacts, joint limits, …), but NOT the DOF-friction
        # constraint force we injected via dof_frictionloss on the previous solve
        # — otherwise the load-dependent friction terms would feed back on
        # themselves. This mirrors bam.mujoco.MujocoController.
        qfrc_bias = self._as_tensor(self._data.qfrc_bias)  # (N, nv)
        qfrc_constraint = self._as_tensor(self._data.qfrc_constraint)  # (N, nv)
        nv = qfrc_bias.shape[-1]
        qfrc_friction = self._dof_friction_force(nv)  # (N, nv)
        external_torque = (
            -qfrc_bias[:, self._dof_ids]
            + qfrc_constraint[:, self._dof_ids]
            - qfrc_friction[:, self._dof_ids]
        )  # (N, J)

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

        return motor_torque


# ─────────────────────────────────────────────────────────────────────────────
# GPU (MuJoCo Warp) simulator — mirrors bam.simulate / bam.mujoco Simulator
# ─────────────────────────────────────────────────────────────────────────────

_ENTITY_NAME = "pendulum"
_JOINT_NAME = "pendulum"


class Simulator:
    """mjlab (MuJoCo Warp) GPU simulator, mirroring :class:`bam.simulate.Simulator`.

    Builds the *same* pendulum spec as :class:`bam.testbench_mujoco.Pendulum`,
    drives the hinge with a :class:`BamActuator` (via :class:`BamActuatorCfg`),
    and steps everything on mjlab's vectorized MuJoCo-Warp pipeline.

    The point of this backend is to exploit mjlab's GPU parallelization: a whole
    batch of logs is rolled out **at once**, one environment per log. Every
    environment shares the same motor model (kt, R, friction budget) but carries
    its own pendulum parameters (``mass``/``arm_mass``/``length``) and firmware
    gains (``kp``/``vin``). This is exactly what makes evaluating a model over a
    directory of logs (as in :mod:`bam.mae`) embarrassingly parallel here.

    How the per-environment "specs" are made different
    --------------------------------------------------
    mjlab replicates one compiled model across all worlds, so per-log physical
    parameters are injected *after* building the sim by expanding the relevant
    model fields per world and writing each environment's values:

    * ``body_mass`` / ``body_ipos`` / ``body_inertia`` — the pendulum inertial,
      computed by :meth:`bam.testbench_mujoco.Pendulum.inertial_params`, followed
      by :meth:`mjlab.sim.Simulation.recompute_constants` to refresh the derived
      mass matrix.
    * ``kp`` — applied through the actuator's per-env ``kp_scale``.
    * ``vin`` — written into the actuator's per-env ``vin_tensor``.

    :param json_path: Path to a BAM params JSON file (produced by ``bam.fit``).
        Mutually exclusive with ``motor_name`` / ``model``.
    :param motor_name: Bundled motor name (e.g. ``"mx106"``). Combine with ``model``.
    :param model: Bundled model variant (``"m1"``–``"m6"``). Combine with ``motor_name``.
    :param device: Torch device, defaults to ``"cuda"`` when available else ``"cpu"``.
    :param integrator: MuJoCo integrator, ``"euler"`` (default, matching the CPU
        backend) or ``"implicitfast"``.
    :param stiff_frictionloss: Forwarded to :class:`BamActuatorCfg`. When ``True``
        (default), the joint-friction constraint is stiffened to counter MuJoCo
        Warp's lack of a noslip solver (see :attr:`BamActuatorCfg.stiff_frictionloss`).
    """

    def __init__(
        self,
        json_path: str | None = None,
        *,
        motor_name: str | None = None,
        model: str | None = None,
        device: str | None = None,
        integrator: str = "euler",
        stiff_frictionloss: bool = True,
    ) -> None:
        self._params_path = _resolve_json_path(json_path, motor_name, model)
        # Default supply voltage from the JSON, used for logs that don't carry vin.
        self._default_vin = load_model(self._params_path).actuator.vin
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.integrator = integrator
        self.stiff_frictionloss = stiff_frictionloss

    # ── Public API (mirrors the reference / CPU simulators) ──────────────────

    def rollout_log(self, log: dict, reset_period: float | None = None) -> tuple:
        """Roll out a single log. See :meth:`rollout_logs`.

        :returns: Tuple ``(positions, velocities, controls)`` — lists over timesteps.
        """
        positions, velocities, controls = self.rollout_logs(
            [log], reset_period=reset_period
        )
        return positions[0], velocities[0], controls[0]

    def rollout_logs(
        self, logs: list[dict], reset_period: float | None = None
    ) -> tuple[list, list, list]:
        """Roll out a batch of logs in parallel — one environment per log.

        All logs must share the same timestep ``dt`` (a single MuJoCo timestep is
        used for the whole batch) but may otherwise differ in pendulum parameters,
        firmware gains, initial state, goal trajectory and length. Logs of
        different lengths are padded (holding the last command) and their outputs
        trimmed back to their own length.

        :param logs: List of processed log dicts.
        :param reset_period: If set, re-synchronize each environment's state to its
            log at this interval [s] (mirrors the reference simulator).
        :returns: Tuple ``(positions, velocities, controls)`` — each a list (one
            entry per log) of per-timestep values. ``controls`` are the applied
            joint torques [Nm] (the BAM actuator drives MuJoCo in motor mode).
        """
        n = len(logs)
        if n == 0:
            return [], [], []

        dts = {log["dt"] for log in logs}
        if len(dts) != 1:
            raise ValueError(
                f"All logs must share the same dt for the mjlab backend, got {sorted(dts)}"
            )
        dt = dts.pop()
        lengths = [len(log["entries"]) for log in logs]
        max_len = max(lengths)

        # ── Pre-extract per-step arrays, padded by holding the last command ──
        goals = np.zeros((max_len, n))
        torque_en = np.zeros((max_len, n), dtype=bool)
        log_pos = np.zeros((max_len, n))
        log_speed = np.zeros((max_len, n))
        for i, log in enumerate(logs):
            entries = log["entries"]
            length = lengths[i]

            def _col(key, default=None):
                return np.array(
                    [e.get(key, default) if default is not None else e[key] for e in entries]
                )

            gi = _col("goal_position")
            ti = _col("torque_enable")
            pi = _col("position")
            si = np.array([e.get("speed", 0.0) for e in entries])
            goals[:length, i], goals[length:, i] = gi, gi[-1]
            torque_en[:length, i], torque_en[length:, i] = ti, ti[-1]
            log_pos[:length, i], log_pos[length:, i] = pi, pi[-1]
            log_speed[:length, i], log_speed[length:, i] = si, si[-1]

        # ── Build the scene / sim / actuator ─────────────────────────────────
        scene, sim, entity, bam_act, body_id = self._build(logs[0], n, dt)
        self._apply_per_env_params(sim, entity, bam_act, body_id, logs)

        dev = self.device
        f32 = torch.float32

        # ── Reset to each log's first entry ──────────────────────────────────
        sim.reset()
        scene.reset()
        p0 = torch.as_tensor(log_pos[0], dtype=f32, device=dev).unsqueeze(1)
        v0 = torch.as_tensor(log_speed[0], dtype=f32, device=dev).unsqueeze(1)
        entity.write_joint_state_to_sim(p0, v0)
        sim.forward()

        ctrl_ids = bam_act.ctrl_ids

        positions = np.zeros((max_len, n))
        velocities = np.zeros((max_len, n))
        controls = np.zeros((max_len, n))

        reset_t = 0.0
        for k in range(max_len):
            # Optional periodic re-sync to the log (mirrors the reference sim).
            reset_t += dt
            if reset_period is not None and reset_t > reset_period:
                reset_t = 0.0
                pk = torch.as_tensor(log_pos[k], dtype=f32, device=dev).unsqueeze(1)
                vk = torch.as_tensor(log_speed[k], dtype=f32, device=dev).unsqueeze(1)
                entity.write_joint_state_to_sim(pk, vk)
                sim.forward()

            positions[k] = entity.data.joint_pos[:, 0].detach().cpu().numpy()
            velocities[k] = entity.data.joint_vel[:, 0].detach().cpu().numpy()

            goal = torch.as_tensor(goals[k], dtype=f32, device=dev).unsqueeze(1)
            entity.set_joint_position_target(goal)
            scene.write_data_to_sim()

            # Zero the applied torque where the log has torque disabled.
            te = torque_en[k]
            if not te.all():
                off = np.nonzero(~te)[0]
                off_ids = torch.as_tensor(off, dtype=torch.long, device=dev)
                zeros = torch.zeros((off_ids.numel(), ctrl_ids.numel()), dtype=f32, device=dev)
                entity.write_ctrl_to_sim(zeros, env_ids=off_ids)

            controls[k] = sim.data.ctrl[:, ctrl_ids][:, 0].detach().cpu().numpy()

            sim.step()
            scene.update(dt=dt)

        # ── Trim each environment's output back to its own length ────────────
        out_pos = [list(positions[: lengths[i], i]) for i in range(n)]
        out_vel = [list(velocities[: lengths[i], i]) for i in range(n)]
        out_ctrl = [list(controls[: lengths[i], i]) for i in range(n)]
        return out_pos, out_vel, out_ctrl

    # ── Internals ────────────────────────────────────────────────────────────

    def _build(self, base_log: dict, num_envs: int, dt: float):
        """Build the scene, sim and actuator for ``num_envs`` pendulums."""
        bam_cfg = BamActuatorCfg(
            json_path=self._params_path,
            target_names_expr=(_JOINT_NAME,),
            kp_fw=float(base_log["kp"]),
            vin=base_log.get("vin"),
            stiff_frictionloss=self.stiff_frictionloss,
        )
        base = {
            "mass": base_log["mass"],
            "arm_mass": base_log.get("arm_mass", 0.0),
            "length": base_log["length"],
        }
        ent_cfg = EntityCfg(
            spec_fn=lambda: Pendulum(base).build_spec(_JOINT_NAME),
            articulation=EntityArticulationInfoCfg(actuators=(bam_cfg,)),
            init_state=EntityCfg.InitialStateCfg(
                joint_pos={_JOINT_NAME: 0.0}, joint_vel={".*": 0.0}
            ),
        )
        scene = Scene(
            SceneCfg(num_envs=num_envs, terrain=None, entities={_ENTITY_NAME: ent_cfg}),
            self.device,
        )
        mj_model = scene.compile()
        # The joint-friction constraint stiffening (for the missing noslip solver)
        # is applied by BamActuator.edit_spec via cfg.stiff_frictionloss.
        sim = Simulation(
            num_envs=num_envs,
            cfg=SimulationCfg(
                mujoco=MujocoCfg(
                    timestep=dt,
                    integrator=self.integrator,
                    gravity=(0.0, 0.0, -Pendulum.G),
                )
            ),
            model=mj_model,
            device=self.device,
        )
        scene.initialize(sim.mj_model, sim.model, sim.data)
        # BamActuator writes per-env friction; those fields must be per-world.
        sim.expand_model_fields(
            ("body_mass", "body_ipos", "body_inertia", "dof_frictionloss", "dof_damping")
        )
        entity = scene[_ENTITY_NAME]
        bam_act = entity.actuators[0]
        body_id = mj_model.body(f"{_ENTITY_NAME}/{_JOINT_NAME}").id
        return scene, sim, entity, bam_act, body_id

    def _apply_per_env_params(self, sim, entity, bam_act, body_id, logs):
        """Inject per-environment pendulum inertial + firmware gains."""
        dev = self.device
        f32 = torch.float32
        n = len(logs)

        masses, coms, inertias = [], [], []
        for log in logs:
            pend = Pendulum(
                {
                    "mass": log["mass"],
                    "arm_mass": log.get("arm_mass", 0.0),
                    "length": log["length"],
                }
            )
            total_mass, com_z, inertia_x = pend.inertial_params()
            masses.append(total_mass)
            coms.append(com_z)
            inertias.append(inertia_x)

        sim.model.body_mass[:, body_id] = torch.as_tensor(masses, dtype=f32, device=dev)
        ipos = sim.model.body_ipos
        ipos[:, body_id, 0] = 0.0
        ipos[:, body_id, 1] = 0.0
        ipos[:, body_id, 2] = torch.as_tensor(coms, dtype=f32, device=dev)
        inertia = sim.model.body_inertia
        it = torch.as_tensor(inertias, dtype=f32, device=dev)
        inertia[:, body_id, 0] = it
        inertia[:, body_id, 1] = it
        inertia[:, body_id, 2] = it
        # Refresh derived quantities (mass matrix, subtree mass) after mass/inertia edits.
        sim.recompute_constants(RecomputeLevel.set_const)

        # Firmware: kp via per-env kp_scale (relative to base kp), vin per-env.
        base_kp = float(logs[0]["kp"])
        kp_scale = torch.as_tensor(
            [[float(log["kp"]) / base_kp] for log in logs], dtype=f32, device=dev
        )
        bam_act.kp_scale[:] = kp_scale
        bam_act.default_kp_scale[:] = kp_scale
        vin = torch.as_tensor(
            [[float(log.get("vin", self._default_vin))] for log in logs],
            dtype=f32,
            device=dev,
        )
        bam_act.vin_tensor[:] = vin
