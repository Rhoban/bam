import numpy as np
import mujoco
from .model import Model


class MujocoController:
    def __init__(
        self,
        model: Model,
        actuator: str,
        mujoco_model: mujoco.MjModel,
        mujoco_data: mujoco.MjData,
    ):
        self.model = model
        self.actuator = np.atleast_1d(actuator)
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data

        # Qpos indexes (qpos)
        self.qpos_indexes = [
            self.mujoco_model.joint(name).qposadr[0] for name in self.actuator
        ]
        # Dof indexes (dof_armature, qvel)
        self.dof_indexes = [
            self.mujoco_model.joint(name).dofadr[0] for name in self.actuator
        ]
        # Actuator indexes (ctrl)
        self.act_indexes = [
            self.mujoco_model.actuator(name).id for name in self.actuator
        ]
        # Joint indexes (efc_id)
        self.joint_indexes = [
            self.mujoco_model.joint(name).id for name in self.actuator
        ]

        # Setting the armature
        self.mujoco_model.dof_armature[self.dof_indexes] = (
            model.actuator.get_extra_inertia()
        )

    def update(self, q_target: float):
        q_target = np.atleast_1d(q_target)
        assert len(q_target) == len(self.actuator), "Invalid target size"

        q = self.mujoco_data.qpos[self.qpos_indexes]
        dq = self.mujoco_data.qvel[self.dof_indexes]

        # Computing the control signal
        control = self.model.actuator.compute_control(q_target - q, q, dq)

        # Computing the applied torque
        torque = self.model.actuator.compute_torque(control, True, q, dq)

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
