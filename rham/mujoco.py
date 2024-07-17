import numpy as np
import mujoco
from .model import Model

EFC_TYPE_FRICTIONLOSS = 1


class MujocoController:
    def __init__(
        self,
        model: Model,
        actuator: str,
        mujoco_model: mujoco.MjModel,
        mujoco_data: mujoco.MjData,
    ):
        self.model = model
        self.actuator = actuator
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data

        self.joint_model = self.mujoco_model.joint(actuator)
        self.joint_model.armature[:] = model.actuator.get_extra_inertia()

        self.joint_data = self.mujoco_data.joint(actuator)
        self.actuator_data = self.mujoco_data.actuator(actuator)

    def update(self, q_target: float):
        q = self.joint_data.qpos[0]
        dq = self.joint_data.qvel[0]

        # Computing the control signal
        control = self.model.actuator.compute_control(q_target - q, q, dq)

        # Computing the applied torque
        torque = self.model.actuator.compute_torque(control, q, dq)

        # Applying the torque
        self.actuator_data.ctrl[:] = torque

        # Updating friction parameters
        torque_external = (
            -self.joint_data.qfrc_bias[0] + self.joint_data.qfrc_constraint[0]
        )

        # Removing all bias forces that were caused by friction
        # See https://github.com/google-deepmind/mujoco/issues/621
        items = np.logical_and(
            self.mujoco_data.efc_id == self.joint_model.id,
            self.mujoco_data.efc_type == EFC_TYPE_FRICTIONLOSS,
        )
        friction_force = sum(self.mujoco_data.efc_force[items])
        torque_external -= friction_force

        # Updating friction parameters
        frictionloss, damping = self.model.compute_frictions(
            self.joint_data.qfrc_actuator[0], torque_external, dq
        )
        self.joint_model.frictionloss[:] = frictionloss
        self.joint_model.damping[:] = damping 

