# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
import mujoco
import json
from copy import copy
from .model import Model, load_model_from_dict
from .testbench_mujoco import Pendulum


class MujocoController:
    """
    A MujocoController is a class allowing to apply the torque and update frictions
    from the computed model during a simulation.

    :param bam.Model model: Model to use (can be loaded using load_model)
    :param str actuator: Actuator to control. The actuated joint properties will be updated. This can be a list of actuators
    :param mujoco.MjModel mujoco_model: The mujoco model
    :param mujoco.MjData mujoco_data: The mujoco data
    :param float | None vin_drop_gain: The voltage drop gain, if not None the voltage will be reduced by 
        vin_drop_gain * load, where load is the sum of the absolute value of the motor torques
    :param float | None vin_min: the minimum voltage, if not None the voltage will not go below this value
    :param float | None max_current: Firmware current limit [A]. If not None, the motor current is
        clipped to ``[-max_current, max_current]`` (equivalently the motor torque is clipped to
        ``±max_current * kt``), reproducing the firmware-side current saturation of the motor.
    """

    def __init__(
        self,
        model: Model,
        actuator: str,
        mujoco_model: mujoco.MjModel,
        mujoco_data: mujoco.MjData,
        vin_drop_gain: float | None = None,
        vin_min: float | None = None,
        max_current: float | None = None,
    ):
        self.model = model
        self.actuator = np.atleast_1d(actuator)
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data
        self.vin_drop_gain = vin_drop_gain
        self.vin_min = vin_min
        self.max_current = max_current

        self.dofs = []
        self.q_target = np.zeros(len(self.actuator))
        self.dof_to_q_target = {}
        for i, name in enumerate(self.actuator):
            self.dof_to_q_target[name] = i

        self.last_ts = mujoco_data.time

        # Actuator indexes (ctrl)
        self.act_indexes = [
            self.mujoco_model.actuator(name).id for name in self.actuator
        ]
        # Joint indexes (efc_id)
        # Retrieved using the first element of the trnid
        self.joint_indexes = [
            self.mujoco_model.actuator(name).trnid[0] for name in self.actuator
        ]
        # Qpos indexes (qpos)
        self.qpos_indexes = self.mujoco_model.jnt_qposadr[self.joint_indexes]
        self.dof_indexes = self.mujoco_model.jnt_dofadr[self.joint_indexes]

        # Setting the armature
        self.mujoco_model.dof_armature[self.dof_indexes] = (
            model.actuator.get_extra_inertia()
        )
        mujoco.mj_setConst(self.mujoco_model, self.mujoco_data)

        self._prev_motor_torque = np.zeros(len(self.actuator))

    def get_q_target(self, name: str) -> float:
        """Return the current target position for a named actuator [rad].

        :param name: Actuator name as passed to the constructor.
        """
        return self.q_target[self.dof_to_q_target[name]]

    def set_q_target(self, name: str, q_target: float):
        """Set the target position for a named actuator.

        :param name: Actuator name as passed to the constructor.
        :param q_target: Desired joint angle [rad].
        """
        self.q_target[self.dof_to_q_target[name]] = q_target
    
    def reset(self, qpos):
        """Reset the controller to a given joint position state.

        Should be called after every ``mujoco.mj_resetData`` to clear the
        internal target and voltage-drop state.

        :param qpos: Full ``mj_data.qpos`` array. The controller extracts
            the positions of its controlled joints.
        """
        self.q_target = qpos[self.qpos_indexes]
        self._prev_motor_torque[:] = 0.0

    def update(self):
        """
        Update the controlled actuator(s) data:
        - Torque to apply
        - Friction parameters
        - Damping
        """
        q = self.mujoco_data.qpos[self.qpos_indexes]
        dq = self.mujoco_data.qvel[self.dof_indexes]

        # Apply voltage drop based on previous step's motor torques
        act = self.model.actuator
        vin_orig = act.vin
        if self.vin_drop_gain is not None:
            load = np.sum(np.abs(self._prev_motor_torque))
            vin_eff = vin_orig - self.vin_drop_gain * load
            if self.vin_min is not None:
                vin_eff = max(vin_eff, self.vin_min)
            act.vin = vin_eff

        # Computing the control signal
        dt = self.mujoco_data.time - self.last_ts
        self.last_ts = self.mujoco_data.time
        control = act.compute_control(self.q_target, q, dq, dt)

        # Computing the applied torque
        torque = act.compute_torque(control, True, q, dq)

        # Firmware current clipping: I = torque / kt is capped at ±max_current,
        # i.e. the motor torque is clipped to ±max_current * kt.
        if self.max_current is not None:
            torque_limit = self.max_current * self.model.kt.value
            torque = np.clip(torque, -torque_limit, torque_limit)

        # Restore original vin and store motor torques for next step's drop computation
        if self.vin_drop_gain is not None:
            act.vin = vin_orig
            self._prev_motor_torque = np.atleast_1d(torque).copy()

        # Applying the torque
        self.mujoco_data.ctrl[self.act_indexes] = torque

        # Updating friction parameters
        torque_external = (
            -self.mujoco_data.qfrc_bias[self.dof_indexes]
            + self.mujoco_data.qfrc_constraint[self.dof_indexes]
        )

        # Repeats the ids (now N_id x N_efc)
        efc_id_repeated = np.repeat(
            [self.mujoco_data.efc_id], len(self.actuator), axis=0
        )
        # Repeat the indexes (now N_id x N_efc)
        id_repeated = np.repeat(
            [self.joint_indexes], len(self.mujoco_data.efc_id), axis=0
        ).T
        # Do the batched test (element wise)
        selector = efc_id_repeated == id_repeated
        # Use * as a logical and
        selector = selector * (
            self.mujoco_data.efc_type == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF.value
        )
        # Sum the forces
        friction_force = np.sum(self.mujoco_data.efc_force * selector, axis=1)

        torque_external -= friction_force
        torque_actuator = self.mujoco_data.qfrc_actuator[self.dof_indexes]

        # Updating friction parameters
        frictionloss, damping = self.model.compute_frictions(
            torque_actuator, torque_external, dq
        )

        # Updating damping and frictionloss
        self.mujoco_model.dof_frictionloss[self.dof_indexes] = frictionloss
        self.mujoco_model.dof_damping[self.dof_indexes] = damping

class Simulator:
    """MuJoCo mirror of :class:`bam.simulate.Simulator`.

    Rolls out a BAM model against the same testbench, but uses MuJoCo physics
    (from :class:`bam.testbench_mujoco.Pendulum`) instead of the hand-written
    Euler integrator, with the actuator driven by a :class:`MujocoController`.

    Vectorization is handled the naive way: one independent ``(MjModel, MjData,
    MujocoController)`` triplet is created per environment and they are ticked
    one by one. This is intentionally not efficient — the point of this
    simulator is to validate that the MuJoCo spec and the controller reproduce
    the reference simulation.

    :param model: BAM friction model to simulate.
    :param actuator: Name used for the hinge joint and the motor actuator in
        the generated spec (and the name the :class:`MujocoController` controls).
    """

    def __init__(self, model: Model, actuator: str = "pendulum"):
        self.model = model
        self.actuator = actuator
        # One entry per environment: (mujoco_model, mujoco_data, controller)
        self.instances: list[tuple] = []
        self.t = 0.0

    def _build_spec(self) -> mujoco.MjSpec:
        testbench = self.model.actuator.testbench
        if testbench is None:
            raise RuntimeError(
                "No testbench set on the actuator. Call model.actuator.load_log(log) "
                "(or set model.actuator.testbench) before building the simulator."
            )
        pendulum = Pendulum(
            {
                "mass": testbench.mass,
                "arm_mass": testbench.arm_mass,
                "length": testbench.length,
            }
        )
        return pendulum.build_spec(self.actuator)

    def reset(self, q: float = 0.0, dq: float = 0.0):
        """(Re)build the environments and reset them to a given state.

        ``q`` and ``dq`` may be scalars or arrays; the number of environments is
        the length of the (broadcast) inputs.

        :param q: Initial joint angle(s) [rad].
        :param dq: Initial joint velocity(ies) [rad/s].
        """
        q = np.atleast_1d(np.asarray(q, dtype=float))
        dq = np.atleast_1d(np.asarray(dq, dtype=float))
        n = max(len(q), len(dq))
        q = np.broadcast_to(q, (n,))
        dq = np.broadcast_to(dq, (n,))

        self.model.reset()
        spec = self._build_spec()

        self.instances = []
        for i in range(n):
            mujoco_model = spec.compile()
            mujoco_data = mujoco.MjData(mujoco_model)
            controller = MujocoController(
                self.model, self.actuator, mujoco_model, mujoco_data
            )
            mujoco_data.qpos[controller.qpos_indexes] = q[i]
            mujoco_data.qvel[controller.dof_indexes] = dq[i]
            mujoco.mj_forward(mujoco_model, mujoco_data)
            controller.reset(mujoco_data.qpos)
            self.instances.append((mujoco_model, mujoco_data, controller))

        self.t = 0.0

    def _pack(self, values: list):
        """Return a scalar for a single environment, else a numpy array."""
        if len(values) == 1:
            return values[0]
        return np.array(values)

    @property
    def q(self):
        """Current joint angle(s) [rad] (scalar if a single environment)."""
        return self._pack(
            [data.qpos[ctrl.qpos_indexes][0] for _, data, ctrl in self.instances]
        )

    @property
    def dq(self):
        """Current joint velocity(ies) [rad/s] (scalar if a single environment)."""
        return self._pack(
            [data.qvel[ctrl.dof_indexes][0] for _, data, ctrl in self.instances]
        )

    def step(self, goal_position, torque_enable, dt: float):
        """Advance every environment by one timestep.

        Unlike :meth:`bam.simulate.Simulator.step` (which is fed a raw control
        signal), the actuator here is a position controller: the input is the
        goal position and the :class:`MujocoController` computes the control and
        the applied torque internally.

        :param goal_position: Target joint angle(s) [rad], scalar or per-environment.
        :param torque_enable: Whether the actuator is powered (scalar or per-env).
            When ``False`` the applied torque is zeroed and only gravity and
            friction act.
        :param dt: Timestep [s].
        """
        goal_position = np.broadcast_to(
            np.atleast_1d(np.asarray(goal_position, dtype=float)),
            (len(self.instances),),
        )
        torque_enable = np.broadcast_to(
            np.atleast_1d(np.asarray(torque_enable)), (len(self.instances),)
        )

        for i, (mujoco_model, mujoco_data, controller) in enumerate(self.instances):
            mujoco_model.opt.timestep = dt
            controller.set_q_target(self.actuator, goal_position[i])
            controller.update()
            if not torque_enable[i]:
                mujoco_data.ctrl[controller.act_indexes] = 0.0
            mujoco.mj_step(mujoco_model, mujoco_data)

        self.t += dt

    def rollout_log(self, log: dict, reset_period: float = None):
        """Roll out the model against a recorded log and return predicted trajectories.

        Mirrors :meth:`bam.simulate.Simulator.rollout_log`, but drives the
        actuator through the :class:`MujocoController` (position control from the
        recorded ``goal_position``), so it is equivalent to the reference
        simulator's ``simulate_control=True`` mode.

        :param log: Processed log dict (see :meth:`bam.logs.Logs.make_batch`).
        :param reset_period: If set, re-synchronize the state to the log at this
            interval [s].
        :returns: Tuple ``(positions, velocities, controls)`` — lists of values
            at each timestep. ``controls`` are the voltages/currents computed by
            the controller.
        """
        positions = []
        velocities = []
        controls = []

        dt = log["dt"]
        self.model.actuator.load_log(log)

        first_entry = log["entries"][0]
        self.reset(
            first_entry["position"],
            first_entry["speed"] if "speed" in first_entry else 0.0,
        )

        reset_period_t = 0.0
        for entry in log["entries"]:
            reset_period_t += dt
            if reset_period is not None and reset_period_t > reset_period:
                reset_period_t = 0.0
                self.reset(entry["position"], entry["speed"])

            positions.append(copy(self.q))
            velocities.append(copy(self.dq))

            # Control recomputed the same way the controller does, for reference.
            control = self.model.actuator.compute_control(
                entry["goal_position"], self.q, self.dq, dt
            )
            controls.append(copy(control))

            self.step(entry["goal_position"], entry["torque_enable"], dt)

        return positions, velocities, controls


def load_config(
    path: str,
    mujoco_model: mujoco.MjModel,
    mujoco_data: mujoco.MjData,
    kp: float,
    vin: float
) -> tuple:
    """
    Loads a BAM configuration file and returns the list of controllers and the mapping dicts.

    Args:
        path (str): path to the configuration file
        mujoco_model (mujoco.MjModel): the mujoco model
        mujoco_data (mujoco.MjData): the mujoco data
        kp (float): the proportional gain
        vin (float): the input voltage

    Returns:
        list: list of controllers, dofs to model mapping, dofs to id mapping
    """
    bam_controllers = {}
    dof_to_bam_controller = {}
    with open(path) as f:
        data = json.load(f)
        for key, value in data.items():
            dofs = value["dofs"]
            for dof in dofs:
                dof_to_bam_controller[dof] = key
                
            model = load_model_from_dict(value["model"])
            model.actuator.kp = kp
            model.actuator.vin = vin
            model.actuator.error_gain = value["error_gain"]
            model.actuator.max_pwm = value["max_pwm"]

            bam_controllers[key] = MujocoController(model, dofs, mujoco_model, mujoco_data)
            bam_controllers[key].dofs = dofs

    return bam_controllers, dof_to_bam_controller
