import os
import numpy as np
import time
import json
import mujoco
import mujoco.viewer
import placo
import pandas
from placo_utils.tf import tf
from rham.model import load_model, Model as rhamModel
from rham.mujoco import MujocoController


class MujocoSimulation2R:
    def __init__(self, testbench: str):
        """
        Loading the 2R simulation
        """
        this_directory = os.path.dirname(os.path.realpath(__file__))

        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(
            f"{this_directory}/2r_{testbench}/scene.xml"
        )
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.testbench = testbench

        # Placo robot
        self.robot = None

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

    def simulate_log(
        self, log: dict, params: str, replay: bool = False, render: bool = False
    ):
        if self.robot is None:
            this_directory = os.path.dirname(os.path.realpath(__file__))
            self.robot = placo.RobotWrapper(
                this_directory + f"/2r_{self.testbench}/robot.urdf", placo.Flags.ignore_collisions
            )
            if self.testbench == "mx106":
                self.robot.set_T_world_frame("base", tf.rotation_matrix(np.pi, [1, 0, 0]))

        # Updating actuator KP
        if "," in params:
            params_r1, params_r2 = params.split(",")
            model_r1, model_r2 = load_model(params_r1), load_model(params_r2)
        else:
            model_r1, model_r2 = load_model(params), load_model(params)

        if type(data["kp"]) is list:
            model_r1.actuator.kp = data["kp"][0]
            model_r2.actuator.kp = data["kp"][1]
        else:
            model_r1.actuator.kp = data["kp"]
            model_r2.actuator.kp = data["kp"]

        # Creating rham controllers
        if not replay:
            r1 = MujocoController(model_r1, "R1", sim.model, sim.data)
            r2 = MujocoController(model_r2, "R2", sim.model, sim.data)

        # Setting initial configuration
        self.data.joint("R1").qpos[0] = data["entries"][0]["r1"]["position"]
        self.data.joint("R2").qpos[0] = data["entries"][0]["r2"]["position"]
        log_t0 = data["entries"][0]["timestamp"]
        self.reset()
        entry_index = 0
        running = True

        while running:
            entry = data["entries"][entry_index]

            self.step()
            if render:
                self.render()

            if replay:
                # If it's a replay, simply jump to the read position
                self.data.joint("R1").qpos[0] = entry["r1"]["position"]
                self.data.joint("R2").qpos[0] = entry["r2"]["position"]
            else:
                r1.update(entry["r1"]["goal_position"])
                r2.update(entry["r2"]["goal_position"])

            while running and (log_t0 + sim.t >= entry["timestamp"]):
                entry = data["entries"][entry_index]
                entry_index += 1
                entry["r1"]["sim_position"] = sim.data.joint("R1").qpos[0]
                entry["r2"]["sim_position"] = sim.data.joint("R2").qpos[0]

                entry["end_effector"] = {}
                for position in "position", "goal_position", "sim_position":
                    self.robot.set_joint("R1", entry["r1"][position])
                    self.robot.set_joint("R2", entry["r2"][position])
                    self.robot.update_kinematics()
                    pos = self.robot.get_T_world_frame("end")[:3, 3]
                    entry["end_effector"][position] = pos

                if entry_index == len(data["entries"]):
                    running = False


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt

    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("--log", type=str, default="2R/log.json", nargs="+")
    args_parser.add_argument("--params", type=str, default="params/mx106/m1.json", nargs="+")
    args_parser.add_argument("--testbench", type=str, required=True)
    args_parser.add_argument("--replay", action="store_true")
    args_parser.add_argument("--render", action="store_true")
    args_parser.add_argument("--plot", action="store_true")
    args_parser.add_argument("--plot_joint", action="store_true")
    args_parser.add_argument("--vertical", action="store_true")
    args_parser.add_argument("--mae", action="store_true")
    args = args_parser.parse_args()

    # Loading rham model
    sim = MujocoSimulation2R(testbench=args.testbench)
    maes = {}

    for log in args.log:
        # Loading log
        data = json.load(open(log))
        maes[log] = {}
        n = len(args.params)

        if args.plot:
            # Creating n horizontal subplots
            if args.vertical:
                f, axs = plt.subplots(n, 1, sharex=True)
            else:
                f, axs = plt.subplots(1, n, sharey=True)

            if n == 1:
                axs = [axs]
            # Setting figure size
            f.set_size_inches(12, 4)
        else:
            axs = [None] * n

        for params, ax in zip(args.params, axs):
            sim.simulate_log(data, params, args.replay, args.render)

            mae = 0
            for dof in "r1", "r2":
                errors = [
                    entry[dof]["position"] - entry[dof]["sim_position"]
                    for entry in data["entries"]
                ]
                mae += np.mean(np.abs(errors))
            mae /= 2
            maes[log][params] = mae

            if args.plot:
                for position in "position", "goal_position", "sim_position":
                    ax.plot(
                        [entry["end_effector"][position][0] for entry in data["entries"]],
                        [entry["end_effector"][position][2] for entry in data["entries"]],
                        label=position,
                        ls="--" if position == "goal_position" else "-",
                    )
                ax.legend()
                ax.grid()
                ax.axis("equal")
                ax.set_title(f"{os.path.basename(log)}, {params}")
                

            if args.plot_joint:
                for dof in "r1", "r2":
                    # Creating two subplots axises
                    f, (ax1, ax2) = plt.subplots(2, sharex=True)

                    goal_positions = [
                        entry[dof]["goal_position"] for entry in data["entries"]
                    ]
                    positions = [entry[dof]["position"] for entry in data["entries"]]
                    sim_positions = [
                        entry[dof]["sim_position"] for entry in data["entries"]
                    ]

                    ax1.plot(goal_positions, label=f"{dof} goal", color="red")
                    ax1.plot(positions, label=f"{dof} read", color="blue")
                    ax1.plot(sim_positions, label=f"{dof} sim", color="green")
                    ax1.grid()
                    ax1.legend()

                    errors = [read - sim for read, sim in zip(positions, sim_positions)]
                    mae = np.mean(np.abs(errors))
                    print("MAE: ", mae)
                    ax2.plot(errors, color="black", label="Simulation error")
                    ax2.set_ylim(-0.05, 0.05)
                    ax2.grid()

                    plt.title(f"{log}, {params}")
                    plt.show()
        
        if args.plot:
            plt.tight_layout()
            plt.show()


    if args.mae:
        total_mae = {params: [] for params in args.params}
        for log in maes:
            print(f"Log: {log}")
            for params in maes[log]:
                print(f"  {params}: {maes[log][params]}")
                total_mae[params] += [maes[log][params]]

        labels = [os.path.basename(log) for log in maes]
        df = pandas.DataFrame(total_mae, index=labels)
        df.plot(kind="bar")
        plt.grid(axis="y")
        # Setting x label with 45°, keeping the top aligned
        plt.xticks(rotation=45, ha="right")
        plt.title("MAE per log")
        plt.tight_layout()
        plt.show()
        
        
        for params in total_mae:
            print(f"Total MAE for {params}: {np.mean(total_mae[params])}")
