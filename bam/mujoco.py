import numpy as np
import mujoco
import json
from .model import Model, load_model_from_dict


class MujocoController:
    """
    A MujocoController is a class allowing to apply the torque and update frictions
    from the computed model during a simulation.

    Args:
        model (Model): model to use (can be loaded using load_model)
        actuator (str): actuator to control
                Note1: the actuated joint properties will be updated
                Note2: this can be a list of actuators
        mujoco_model (mujoco.MjModel): the mujoco model
        mujoco_data (mujoco.MjData): the mujoco data
    """

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

    def update(self, q_target: float):
        """
        Update the controlled actuator(s) data:
        - Torque to apply
        - Friction parameters
        - Damping

        Args:
            q_target (float): target position for the actuator(s)
        """
        q_target = np.atleast_1d(q_target)
        assert len(q_target) == len(self.actuator), "Invalid target size"

        q = self.mujoco_data.qpos[self.qpos_indexes]
        dq = self.mujoco_data.qvel[self.dof_indexes]

        # Computing the control signal
        dt = self.mujoco_data.time - self.last_ts
        self.last_ts = self.mujoco_data.time
        control = self.model.actuator.compute_control(q_target, q, dq, dt)

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


def load_config(path: str, mujoco_model: mujoco.MjModel, mujoco_data: mujoco.MjData, kp: float, vin: float, error_gain: float, max_pwm: float) -> tuple:
    """
    Loads a BAM configuration file and returns the list of controllers and the mapping dicts.

    Args:
        path (str): path to the configuration file
        mujoco_model (mujoco.MjModel): the mujoco model
        mujoco_data (mujoco.MjData): the mujoco data
        kp (float): the proportional gain
        vin (float): the input voltage
        error_gain (float): the error gain
        max_pwm (float): the maximum pwm

    Returns:
        list: list of controllers, dofs to model mapping, dofs to id mapping
    """
    controllers = {}
    dofs_to_model = {}
    dofs_to_ctrl_id = {}
    with open(path) as f:
        data = json.load(f)
        for key, value in data.items(): 
            dofs = value["dofs"]
            for i, dof in enumerate(dofs):
                dofs_to_model[dof] = key
                dofs_to_ctrl_id[dof] = i

            model = load_model_from_dict(value["model"])
            model.actuator.kp = kp
            model.actuator.vin = vin
            model.actuator.error_gain = error_gain
            model.actuator.max_pwm = max_pwm
            
            controller = MujocoController(model, dofs, mujoco_model, mujoco_data)
            controllers[key] = {"controller": controller,
                                "dofs_state": [0.] * len(dofs)}
            
    return controllers, dofs_to_model, dofs_to_ctrl_id