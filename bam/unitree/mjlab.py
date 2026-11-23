# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Unitree J288 actuator for mjlab (MuJoCo Warp).

:class:`J288BamActuator` runs the J288 firmware law of
:class:`bam.unitree.actuator.J288Actuator` vectorized over ``(num_envs, num_joints)``
and reuses :class:`bam.mjlab.BamActuator` for everything else: the friction
budget written into MuJoCo, battery voltage and sag, command delay, friction
scaling. What it adds:

* the full hybrid command: ``position_target``, ``velocity_target`` (the
  firmware ``w_des``) and ``effort_target`` (``tau_ff``) from mjlab's command;
* per-environment, per-joint ``kp`` and ``kd`` that a policy or an event can set
  at any step (:meth:`J288BamActuator.set_joint_gains`), on top of the parent's
  per-environment ``kp_scale`` / ``kd_scale`` randomization (here ``kd_scale``
  scales the firmware damping gain, not back-EMF);
* mode 0 per joint (:meth:`J288BamActuator.set_enabled`), which brakes;
* :attr:`J288BamActuator.reported_torque` and
  :attr:`J288BamActuator.reported_speed`, the values the servo sends back, for
  observations.

Usage::

    from bam.unitree.mjlab import J288BamActuatorCfg

    actuators = J288BamActuatorCfg(
        json_path="bam/params/unitree_j288/m6.json",
        target_names_expr=(r".*",),
        kp_fw=0.97,
        kd_fw=0.02,
        vin_range=(21.0, 25.2),
    )

:class:`J288Simulator` rolls J288 logs out on this actuator, like
:class:`bam.mjlab.Simulator` does for the voltage-controlled servos, to check a
fitted model on the mjlab backend and at a training timestep.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import torch

from mjlab.actuator.actuator import ActuatorCmd
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.managers.event_manager import RecomputeLevel
from mjlab.scene import Scene, SceneCfg
from mjlab.sim import MujocoCfg, Simulation, SimulationCfg

from bam.mjlab import _ENTITY_NAME, _JOINT_NAME, BamActuator, BamActuatorCfg
from bam.mjlab import Simulator as BamSimulator
from bam.simulate import fractional_delay_shift
from bam.testbench_mujoco import Pendulum
from bam.unitree.actuator import J288Actuator

if TYPE_CHECKING:
    from mjlab.entity import Entity


@dataclass(kw_only=True)
class J288BamActuatorCfg(BamActuatorCfg):
    """Configuration for :class:`J288BamActuator`.

    Takes every :class:`bam.mjlab.BamActuatorCfg` option. ``kp_fw`` and
    ``kd_fw`` are the default gains, output side, in N.m/rad and N.m.s/rad.
    """

    kd_fw: float | None = None

    def build(
        self, entity: "Entity", target_ids: list[int], target_names: list[str]
    ) -> "J288BamActuator":
        return J288BamActuator(self, entity, target_ids, target_names)


class J288BamActuator(BamActuator):
    """J288 servo in mjlab. See the module documentation."""

    cfg: J288BamActuatorCfg

    def __init__(self, cfg, entity, target_ids, target_names) -> None:
        super().__init__(cfg, entity, target_ids, target_names)
        act = self._bam_model.actuator
        if not isinstance(act, J288Actuator):
            raise TypeError(
                f"J288BamActuator needs a j288 parameter file, got {type(act).__name__}"
            )
        if cfg.kd_fw is not None:
            act.kd = cfg.kd_fw

        self.kp: torch.Tensor | None = None
        self.kd: torch.Tensor | None = None
        self.enabled: torch.Tensor | None = None
        self.reported_torque: torch.Tensor | None = None
        self.reported_speed: torch.Tensor | None = None

    def initialize(self, mj_model, model, data, device) -> None:
        super().initialize(mj_model, model, data, device)
        act = self._bam_model.actuator
        shape = (self._num_envs, len(self._dof_ids))
        f32 = torch.float32
        self.kp = torch.full(shape, float(act.kp), dtype=f32, device=device)
        self.kd = torch.full(shape, float(act.kd), dtype=f32, device=device)
        self.default_kp = self.kp.clone()
        self.default_kd = self.kd.clone()
        self.enabled = torch.ones(shape, dtype=torch.bool, device=device)
        self.reported_torque = torch.zeros(shape, dtype=f32, device=device)
        self.reported_speed = torch.zeros(shape, dtype=f32, device=device)

    # ── Commands ────────────────────────────────────────────────────────────

    def set_joint_gains(
        self,
        env_ids: torch.Tensor | slice,
        kp: torch.Tensor | float | None = None,
        kd: torch.Tensor | float | None = None,
    ) -> None:
        """Set the firmware gains [N.m/rad, N.m.s/rad] per env and joint.

        Values broadcast to ``(len(env_ids), num_joints)``. They are rounded to
        the protocol's resolution when applied, as the servo would. The
        per-environment ``kp_scale`` / ``kd_scale`` still multiply them.
        """
        if kp is not None:
            self.kp[env_ids] = torch.as_tensor(
                kp, dtype=self.kp.dtype, device=self.kp.device
            )
        if kd is not None:
            self.kd[env_ids] = torch.as_tensor(
                kd, dtype=self.kd.dtype, device=self.kd.device
            )

    def set_enabled(self, env_ids: torch.Tensor | slice, enabled) -> None:
        """Mode 1 (``True``) or mode 0, shorted-winding braking (``False``)."""
        self.enabled[env_ids] = torch.as_tensor(
            enabled, dtype=torch.bool, device=self.enabled.device
        )

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        if self.enabled is not None:
            self.enabled[ids] = True

    # ── Main compute ────────────────────────────────────────────────────────

    def compute(self, cmd: ActuatorCmd) -> torch.Tensor:
        """J288 torques for all environments, shape ``(N, J)``.

        Mirrors :meth:`bam.mjlab.BamActuator.compute`, with the J288 firmware law
        in place of the P controller.
        """
        bam = self._bam_model
        act: J288Actuator = bam.actuator
        assert self._data is not None and self._dof_ids is not None

        prev_actuator_torque = self._as_tensor(self._data.qfrc_actuator)[
            :, self._dof_ids
        ]

        # Supply voltage with the optional battery drop (as the parent).
        vin = self.vin_tensor
        duty_cycle = act.duty_cycle
        if self.vin_drop_resistance is not None and duty_cycle is not None:
            current = (duty_cycle * prev_actuator_torque / bam.kt.value).sum(
                dim=-1, keepdim=True
            )
            current = torch.clamp(current, min=0.0)
            vin = vin - self.vin_drop_resistance * current
            if self.cfg.vin_min is not None:
                vin = torch.clamp(vin, min=self.cfg.vin_min)

        # Firmware law: commands and gains as actuator attributes.
        act.vin = vin
        act.kp = self.kp * self.kp_scale
        act.kd = self.kd * self.kd_scale
        act.tau_ff = cmd.effort_target
        act.w_des = cmd.velocity_target
        act.enabled = self.enabled
        control = act.compute_control(cmd.position_target, cmd.pos, cmd.vel, self._dt)
        motor_torque = act.compute_torque(control, True, cmd.pos, cmd.vel)
        self.reported_torque = act.reported_torque
        self.reported_speed = act.w_filtered

        # Friction budget, exactly as the parent.
        qfrc_bias = self._as_tensor(self._data.qfrc_bias)
        qfrc_constraint = self._as_tensor(self._data.qfrc_constraint)
        nv = qfrc_bias.shape[-1]
        qfrc_friction = self._dof_friction_force(nv)
        external_torque = (
            -qfrc_bias[:, self._dof_ids]
            + qfrc_constraint[:, self._dof_ids]
            - qfrc_friction[:, self._dof_ids]
        )
        vel = cmd.vel
        if bam.stribeck:
            stribeck_coeff = torch.exp(
                -torch.pow(torch.abs(vel) / bam.dtheta_stribeck.value, bam.alpha.value)
            )
        else:
            stribeck_coeff = torch.zeros_like(vel)
        frictionloss = self._compute_friction_budget(
            prev_actuator_torque, external_torque, stribeck_coeff
        )
        self._write_frictions(frictionloss, bam.friction_viscous.value)

        return motor_torque


# ─────────────────────────────────────────────────────────────────────────────
# Log rollouts on mjlab
# ─────────────────────────────────────────────────────────────────────────────


class J288Simulator(BamSimulator):
    """Roll J288 logs out on :class:`J288BamActuator`.

    Same interface as :class:`bam.mjlab.Simulator`, but replays the full J288
    command per step (``goal_position``, ``goal_speed``, ``goal_torque``,
    ``kp``, ``kd``, ``torque_enable``) and returns the simulated reported torque
    as ``controls``. Torque-off steps are mode 0 (braking), not zero torque.
    ``dt`` sets the MuJoCo timestep: resample the logs to a training timestep
    (``bam.process --dt``) to check the model at that step.
    """

    COMMANDS = (
        "goal_position",
        "goal_speed",
        "goal_torque",
        "kp",
        "kd",
        "torque_enable",
    )

    def rollout_logs(self, logs: list[dict], reset_period: float | None = None):
        n = len(logs)
        if n == 0:
            return [], [], []
        dts = {log["dt"] for log in logs}
        if len(dts) != 1:
            raise ValueError(f"All logs must share the same dt, got {sorted(dts)}")
        dt = dts.pop()
        lengths = [len(log["entries"]) for log in logs]
        max_len = max(lengths)

        def column(key, default):
            out = np.zeros((max_len, n))
            for i, log in enumerate(logs):
                entries = log["entries"]
                fallback = log.get(key, default)
                values = np.array([e.get(key, fallback) for e in entries], dtype=float)
                out[: lengths[i], i], out[lengths[i] :, i] = values, values[-1]
            return out

        cols = {
            "goal_position": column("goal_position", 0.0),
            "goal_speed": column("goal_speed", 0.0),
            "goal_torque": column("goal_torque", 0.0),
            "kp": column("kp", 0.0),
            "kd": column("kd", 0.0),
            "torque_enable": column("torque_enable", 1.0),
        }
        log_pos = column("position", 0.0)
        log_speed = column("speed", 0.0)
        if self.command_delay and self._command_delay_value > 0.0:
            for key in ("goal_position", "goal_speed", "goal_torque", "kp", "kd"):
                cols[key] = fractional_delay_shift(
                    cols[key], self._command_delay_value, dt
                )

        scene, sim, entity, act, body_id = self._build(logs[0], n, dt)
        self._apply_per_env_params(sim, entity, act, body_id, logs)

        dev, f32 = self.device, torch.float32

        def t(x):
            return torch.as_tensor(x, dtype=f32, device=dev).unsqueeze(1)

        sim.reset()
        scene.reset()
        entity.write_joint_state_to_sim(t(log_pos[0]), t(log_speed[0]))
        sim.forward()

        positions = np.zeros((max_len, n))
        velocities = np.zeros((max_len, n))
        controls = np.zeros((max_len, n))
        reset_t = 0.0
        for k in range(max_len):
            reset_t += dt
            if reset_period is not None and reset_t > reset_period:
                reset_t = 0.0
                entity.write_joint_state_to_sim(t(log_pos[k]), t(log_speed[k]))
                sim.forward()

            positions[k] = entity.data.joint_pos[:, 0].detach().cpu().numpy()
            velocities[k] = entity.data.joint_vel[:, 0].detach().cpu().numpy()

            entity.set_joint_position_target(t(cols["goal_position"][k]))
            entity.set_joint_velocity_target(t(cols["goal_speed"][k]))
            entity.set_joint_effort_target(t(cols["goal_torque"][k]))
            act.set_joint_gains(slice(None), kp=t(cols["kp"][k]), kd=t(cols["kd"][k]))
            act.set_enabled(slice(None), t(cols["torque_enable"][k]) > 0.5)
            scene.write_data_to_sim()  # runs the actuator
            controls[k] = act.reported_torque[:, 0].detach().cpu().numpy()

            sim.step()
            scene.update(dt=dt)

        return (
            [list(positions[: lengths[i], i]) for i in range(n)],
            [list(velocities[: lengths[i], i]) for i in range(n)],
            [list(controls[: lengths[i], i]) for i in range(n)],
        )

    def _build(self, base_log: dict, num_envs: int, dt: float):
        """As :meth:`bam.mjlab.Simulator._build`, with the J288 actuator."""
        act_cfg = J288BamActuatorCfg(
            json_path=self._params_path,
            target_names_expr=(_JOINT_NAME,),
            kp_fw=float(base_log.get("kp", 0.0)),
            kd_fw=float(base_log.get("kd", 0.0)),
            vin=base_log.get("vin"),
            stiff_frictionloss=self.stiff_frictionloss,
        )
        base = {
            "mass": base_log["mass"],
            "arm_mass": base_log.get("arm_mass", 0.0),
            "length": base_log["length"],
        }
        ent_cfg = EntityCfg(
            spec_fn=lambda: Pendulum(base).build_spec(
                _JOINT_NAME, q_offset=self._q_offset
            ),
            articulation=EntityArticulationInfoCfg(actuators=(act_cfg,)),
            init_state=EntityCfg.InitialStateCfg(
                joint_pos={_JOINT_NAME: 0.0}, joint_vel={".*": 0.0}
            ),
        )
        scene = Scene(
            SceneCfg(num_envs=num_envs, terrain=None, entities={_ENTITY_NAME: ent_cfg}),
            self.device,
        )
        mj_model = scene.compile()
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
        sim.expand_model_fields(
            (
                "body_mass",
                "body_ipos",
                "body_inertia",
                "dof_frictionloss",
                "dof_damping",
            )
        )
        entity = scene[_ENTITY_NAME]
        act = entity.actuators[0]
        body_id = mj_model.body(f"{_ENTITY_NAME}/{_JOINT_NAME}").id
        return scene, sim, entity, act, body_id

    def _apply_per_env_params(self, sim, entity, act, body_id, logs):
        """Per-env pendulum inertial (including ``mass_inertia``) and supply
        voltage; the gains are written per step by :meth:`rollout_logs`."""
        dev, f32 = self.device, torch.float32
        masses, coms, inertias = [], [], []
        for log in logs:
            pend = Pendulum(
                {
                    "mass": log["mass"],
                    "arm_mass": log.get("arm_mass", 0.0),
                    "length": log["length"],
                }
            )
            total_mass, com_z, inertia = pend.inertial_params()
            masses.append(total_mass)
            coms.append(com_z)
            inertias.append(inertia + log.get("mass_inertia", 0.0))
        sim.model.body_mass[:, body_id] = torch.as_tensor(masses, dtype=f32, device=dev)
        ipos = sim.model.body_ipos
        ipos[:, body_id, 0] = 0.0
        ipos[:, body_id, 1] = 0.0
        ipos[:, body_id, 2] = torch.as_tensor(coms, dtype=f32, device=dev)
        inertia = sim.model.body_inertia
        it = torch.as_tensor(inertias, dtype=f32, device=dev)
        for axis in range(3):
            inertia[:, body_id, axis] = it
        sim.recompute_constants(RecomputeLevel.set_const)
        act.vin_tensor[:] = torch.as_tensor(
            [[float(log.get("vin", self._default_vin))] for log in logs],
            dtype=f32,
            device=dev,
        )
