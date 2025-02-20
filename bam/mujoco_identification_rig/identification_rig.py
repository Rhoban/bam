import mujoco
import numpy as np
from threading import Thread

import mujoco.viewer
import time

from bam.model import load_model
from bam.mujoco import MujocoController


class MujocoIdentificationRig:
    def __init__(self, bam=False, model_path=None):
        self.bam = bam
        if self.bam and model_path is None:
            print("Bam mode requires model_path to be set")
            exit()
            
        if self.bam:
            self.model = mujoco.MjModel.from_xml_path(
                "bam/mujoco_identification_rig/assets/identification_rig_0_150m_1kg/scene_motor.xml"
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(
                "bam/mujoco_identification_rig/assets/identification_rig_0_150m_1kg/scene.xml"
            )
        self.model.opt.timestep = 0.002
        self.control_decimation = 10
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)

        if self.bam:
            sts3215_model = load_model(model_path)
            self.mujoco_controller = MujocoController(
                sts3215_model, "sts3215", self.model, self.data
            )


        # self.kp = self.data.

        self.goal_position = 0
        self.torque_enabled = True
        Thread(target=self.run).start()
        self.ready = True

    def enable_torque(self):
        self.torque_enabled = True

    def disable_torque(self):
        self.torque_enabled = False

    def set_goal_position(self, position):
        self.goal_position = position

    def get_present_position(self):
        return self.data.qpos[0]

    def get_present_velocity(self):
        return self.data.qvel[0]

    def run(self):
        with mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            counter = 0
            while True:

                step_start = time.time()  # Was

                mujoco.mj_step(self.model, self.data)

                counter += 1
                if counter % self.control_decimation == 0:
                    if self.bam:
                        self.mujoco_controller.update(self.goal_position)
                    else:
                        self.data.ctrl = self.goal_position

                viewer.sync()

                # Was
                time_until_next_step = self.model.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)


if __name__ == "__main__":
    m = MujocoIdentificationRig(bam=True, model_path="data/brutal_no_load_sts3215/params_m1.json")
    while True:
        m.set_goal_position(0.5 * np.sin(2 * np.pi * 1.5 * time.time()))
        print(m.get_present_velocity())
        time.sleep(0.01)
