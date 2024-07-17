import os
import numpy as np
import time
import json
import mujoco
import mujoco.viewer
import placo
from placo_utils.tf import tf
from rham.model import load_model
from rham.mujoco import MujocoController


class MujocoSimulation2R:
    def __init__(self):
        """
        Loading the 2R simulation
        """
        this_directory = os.path.dirname(os.path.realpath(__file__))

        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(
            f"{this_directory}/sw_106/scene.xml"
        )
        self.data: mujoco.MjData = mujoco.MjData(self.model)

        self.viewer = None
        self.viewer_start = None
        self.t: float = 0
        self.dt: float = self.model.opt.timestep
        self.frame: int = 0

    def step(self) -> None:
        self.t = self.frame * self.dt
        mujoco.mj_step(self.model, self.data)
        self.frame += 1

    def reset(self):
        self.t = 0
        self.frame = 0
        self.viewer_start = time.time()

    def render(self, realtime: bool = True):
        """
        Renders the visualization of the simulation.

        Args:
            realtime (bool, optional): if True, render will sleep to ensure real time viewing. Defaults to True.
        """
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer_start = time.time()

        if realtime:
            current_ts = self.viewer_start + self.frame * self.dt
            to_sleep = current_ts - time.time()
            if to_sleep > 0:
                time.sleep(to_sleep)

        self.viewer.sync()


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt

    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("--log", type=str, default="2R/log.json")
    args_parser.add_argument("--params", type=str, default="params/mx106/m1.json")
    args_parser.add_argument("--replay", action="store_true")
    args_parser.add_argument("--render", action="store_true")
    args_parser.add_argument("--loop", action="store_true")
    args_parser.add_argument("--plot", action="store_true")
    args = args_parser.parse_args()

    sim = MujocoSimulation2R()
    model = load_model(args.params)

    this_directory = os.path.dirname(os.path.realpath(__file__))
    robot = placo.RobotWrapper(this_directory + "/sw_106/robot.urdf", placo.Flags.ignore_collisions)
    robot.set_T_world_frame("base", tf.rotation_matrix(np.pi, [1, 0, 0]))

    data = json.load(open(args.log))
    model.actuator.kp = data["kp"]
    print(f"Using kp: {model.actuator.kp}")
    log_t0 = data["entries"][0]["timestamp"]

    if not args.replay:
        r1 = MujocoController(model, "R1", sim.model, sim.data)
        r2 = MujocoController(model, "R2", sim.model, sim.data)
    mujoco.mj_resetData(sim.model, sim.data)

    while True:
        sim.reset()
        entry_index = 0
        sim.data.joint("R1").qpos[0] = data["entries"][0]["r1"]["position"]
        sim.data.joint("R2").qpos[0] = data["entries"][0]["r2"]["position"]
        running = True

        while running:
            entry = data["entries"][entry_index]

            sim.step()
            if args.render:
                sim.render()

            if args.replay:
                sim.data.joint("R1").qpos[0] = entry["r1"]["position"]
                sim.data.joint("R2").qpos[0] = entry["r2"]["position"]
            else:
                r1.update(entry["r1"]["goal_position"])
                r2.update(entry["r2"]["goal_position"])

            if log_t0 + sim.t >= entry["timestamp"]:
                entry_index += 1
                entry["r1"]["sim_position"] = sim.data.joint("R1").qpos[0]
                entry["r2"]["sim_position"] = sim.data.joint("R2").qpos[0]

                entry["end_effector"] = {}
                for position in "position", "goal_position", "sim_position":
                    robot.set_joint("R1", entry["r1"][position])
                    robot.set_joint("R2", entry["r2"][position])
                    robot.update_kinematics()
                    pos = robot.get_T_world_frame("end")[:3, 3]
                    entry["end_effector"][position] = pos

                if entry_index == len(data["entries"]):
                    running = False

        if args.plot:
            for position in "position", "goal_position", "sim_position":
                plt.plot(
                    [entry["end_effector"][position][0] for entry in data["entries"]],
                    [entry["end_effector"][position][2] for entry in data["entries"]],
                    label=position,
                )
            plt.legend()
            plt.grid()
            plt.axis("equal")
            plt.show()            

            for dof in "r1", "r2":
                # Creating two subplots axises
                f, (ax1, ax2) = plt.subplots(2, sharex=True)

                goal_positions = [entry[dof]["goal_position"] for entry in data["entries"]]
                positions = [entry[dof]["position"] for entry in data["entries"]]
                sim_positions = [entry[dof]["sim_position"] for entry in data["entries"]]

                ax1.plot(goal_positions, label=f"{dof} goal", color="red")
                ax1.plot(positions, label=f"{dof} read", color="blue")
                ax1.plot(sim_positions, label=f"{dof} sim", color="green")
                ax1.grid()
                ax1.legend()

                errors = [read - sim for read, sim in zip(positions, sim_positions)]
                ax2.plot(errors, color="black", label="Simulation error")
                ax2.set_ylim(-0.05, 0.05)
                ax2.grid()

                plt.title(args.params)
                plt.show()

        if not args.loop:
            break
