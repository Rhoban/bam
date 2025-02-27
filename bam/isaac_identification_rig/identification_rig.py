#!/usr/bin/env python3


# Isaac self.gym imports
import time
from threading import Thread
from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch

import numpy as np

MASS = "0_5"  # 0_5, 1
LEVER_LENGTH = "0_150"  # 0_100, 0_150

ARMATURE = 0.027
MAX_EFFORT = 3.57
KP = 6.55
FRICTION = 0.045
# FRICTION = 0.02
DAMPING = 0.65


class IsaacIdentificationRig:
    def __init__(self):
        self.ready = False
        # 1. Parse arguments (Isaac self.gym utility function)
        args = gymutil.parse_arguments(description="Isaac self.gym Boilerplate Example")

        # 2. Acquire the self.gym interface
        self.gym = gymapi.acquire_gym()

        # 3. Configure the simulation
        self.sim_params = gymapi.SimParams()
        self.sim_params.up_axis = gymapi.UP_AXIS_Z  # Z-up coordinate system
        self.sim_params.gravity.x = 0
        self.sim_params.gravity.y = 0
        self.sim_params.gravity.z = -9.81
        self.sim_params.dt = 0.005  # simulation timestep
        self.sim_params.substeps = 1  # physics substeps
        self.control_decimation = 4
        self.dt = self.sim_params.dt * self.control_decimation

        # You can switch to PhysX or Flex depending on your installation
        self.sim_params.physx.solver_type = 0 # 0: pgs, 1: tgs
        # self.sim_params.physx.num_position_iterations = 4
        # self.sim_params.physx.num_velocity_iterations = 1
        # self.sim_params.physx.contact_offset = 0.02
        # self.sim_params.physx.rest_offset = 0.0
        # self.sim_params.physx.bounce_threshold_velocity = 0.2
        # self.sim_params.physx.max_depenetration_velocity = 100.0
        # self.sim_params.physx.default_buffer_size_multiplier = 5.0
        # self.sim_params.physx.contact_collection = 1
        self.sim_params.physx.use_gpu = (
            True  # set to False if you don't have GPU support
        )

        # 4. Create the simulation (use GPU device 0, graphics device 0, PhysX, etc.)
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, self.sim_params)
        if self.sim is None:
            raise Exception("Failed to create sim")

        # 5. Add a ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        self.gym.add_ground(self.sim, plane_params)

        # 6. Create a viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            raise Exception("Failed to create viewer")

        # 7. Create (or load) assets and environments
        #    We'll just make one environment here as an example
        self.envs = []
        self.num_envs = 1

        # The spacing below is how far apart multiple envs would be placed if you had more than one
        env_lower = gymapi.Vec3(-1.0, -1.0, 0.0)
        env_upper = gymapi.Vec3(1.0, 1.0, 1.0)

        # Path where your URDF or mesh files exist
        asset_root = "./bam/isaac_identification_rig/assets"
        # asset_root = "./assets"
        asset_file = f"identification_rig_{LEVER_LENGTH}m_{MASS}kg/robot.urdf"

        # Setup asset options
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.disable_gravity = False
        asset_options.thickness = 0.00
        asset_options.density = 0.000
        asset_options.angular_damping = 0.00
        asset_options.linear_damping = 0.0
        asset_options.max_angular_velocity = 1000.0
        asset_options.max_linear_velocity = 1000.0
        asset_options.default_dof_drive_mode = 3

        # Load the asset
        self.asset = self.gym.load_asset(
            self.sim, asset_root, asset_file, asset_options
        )

        for i in range(self.num_envs):
            # Create an environment
            env = self.gym.create_env(self.sim, env_lower, env_upper, self.num_envs)
            self.envs.append(env)

            # Create an actor (the robot)
            pose = gymapi.Transform()
            pose.p.x = 0.0
            pose.p.y = 0.0
            pose.p.z = 0.1

            # Add the robot to the environment
            # The last two parameters: "franka" is the name for the actor, and i is the index
            self.handle = self.gym.create_actor(env, self.asset, pose, "actor", i, 1)

            dof_props = self.gym.get_asset_dof_properties(self.asset)
            dof_props["friction"] = FRICTION
            dof_props["armature"] = ARMATURE
            # dof_props["damping"] = DAMPING
            # print(dof_props)
            # exit()

            self.gym.set_actor_dof_properties(env, self.handle, dof_props)

        self.dof_state_tensor = self.gym.acquire_dof_state_tensor(self.sim)
        self.dof_state = gymtorch.wrap_tensor(self.dof_state_tensor)
        self.mass_matrix_tensor = self.gym.acquire_mass_matrix_tensor(self.sim, "actor")
        self.mass_matrix_state = gymtorch.wrap_tensor(self.mass_matrix_tensor)
        self.dofs_per_env = self.dof_state.shape[0] // self.num_envs
        self.dof_pos = self.dof_state.view(self.num_envs, self.dofs_per_env, 2)[
            ..., :1, 0
        ]
        self.dof_vel = self.dof_state.view(self.num_envs, self.dofs_per_env, 2)[
            ..., :1, 1
        ]
        self.net_contact_force_tensor = self.gym.acquire_net_contact_force_tensor(self.sim)
        self.net_contact_force = gymtorch.wrap_tensor(self.net_contact_force_tensor)
        self.dof_force_tensor = self.gym.acquire_dof_force_tensor(self.sim)
        self.dof_force = gymtorch.wrap_tensor(self.dof_force_tensor)


        self.kp = KP
        self.kd = DAMPING
        # self.kd = 0
        self.max_effort = MAX_EFFORT
        self.torque_enabled = True
        self.goal_position = 0
        Thread(target=self.run).start()

    def set_friction(self, friction):

        for i in range(self.num_envs):
            env = self.envs[i]

            dof_props = self.gym.get_asset_dof_properties(self.asset)
            dof_props["friction"] = friction

            self.gym.set_actor_dof_properties(env, self.handle, dof_props)
        pass

    def set_damping(self, damping):
        self.kd = damping

    def set_kp(self, kp):
        self.kp = kp

    def run(self):
        self.ready = True
        while not self.gym.query_viewer_has_closed(self.viewer):
            for _ in range(self.control_decimation):
                torques = self.kp * (
                    self.goal_position - self.dof_pos
                )  # - (self.kd * self.dof_vel)
                torques = np.clip(torques, -self.max_effort, self.max_effort)
                torques *= self.torque_enabled
                torques -= self.kd * self.dof_vel
                self.gym.set_dof_actuation_force_tensor(
                    self.sim, gymtorch.unwrap_tensor(torques)
                )

                self.gym.simulate(self.sim)
                self.gym.fetch_results(self.sim, True)
                self.gym.refresh_dof_state_tensor(self.sim)

            self.gym.refresh_dof_state_tensor(self.sim)
            self.gym.refresh_actor_root_state_tensor(self.sim)
            self.gym.refresh_rigid_body_state_tensor(self.sim)
            self.gym.refresh_mass_matrix_tensors(self.sim)
            # self.gym.refresh_dof_force_tensor(self.sim)

            self.gym.refresh_force_sensor_tensor(self.sim)
            self.gym.refresh_dof_force_tensor(self.sim)
            self.gym.refresh_net_contact_force_tensor(self.sim)
            self.gym.step_graphics(self.sim)
            
            print(self.dof_force.shape)
            print(self.dof_force)
            print("===")

            self.gym.draw_viewer(self.viewer, self.sim, True)
            self.gym.sync_frame_time(self.sim)

        # 9. Cleanup
        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)

    def enable_torque(self):
        self.torque_enabled = True

    def disable_torque(self):
        self.torque_enabled = False

    def set_goal_position(self, position):
        self.goal_position = position

    def get_present_position(self):
        return self.dof_pos[0][0].cpu().numpy()

    def get_present_velocity(self):
        return self.dof_vel[0][0].cpu().numpy()


if __name__ == "__main__":
    i = IsaacIdentificationRig()
    A = 0.5
    F = 1.5
    s = time.time()
    set = False
    i.disable_torque()
    while True:
        # i.set_goal_position(A * np.sin(2 * np.pi * F * time.time()))
        # print(i.get_present_position())
        # print(i.get_present_velocity())
        time.sleep(0.01)
        # if time.time() - s > 5 and not set:
        #     print("a")
        #     i.set_friction(0.3)
        #     set = True

