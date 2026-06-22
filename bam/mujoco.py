# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
import mujoco
import json
from .model import Model, load_model_from_dict


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
    """

    def __init__(
        self,
        model: Model,
        actuator: str,
        mujoco_model: mujoco.MjModel,
        mujoco_data: mujoco.MjData,
        vin_drop_gain: float | None = None,
        vin_min: float | None = None,
    ):
        self.model = model
        self.actuator = np.atleast_1d(actuator)
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data
        self.vin_drop_gain = vin_drop_gain
        self.vin_min = vin_min

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
        return self.q_target[self.dof_to_q_target[name]]
    
    def set_q_target(self, name: str, q_target: float):
        self.q_target[self.dof_to_q_target[name]] = q_target
    
    def reset(self, qpos):
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
