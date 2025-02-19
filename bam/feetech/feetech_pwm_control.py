from pypot.feetech import FeetechSTS3215IO
import time
import numpy as np
from threading import Thread


# Must stay in degrees to match pypot default.
class FeetechPWMControl:
    def __init__(self, id, kp=None):
        self.io = FeetechSTS3215IO(
            "/dev/ttyACM0",
            baudrate=1000000,
            use_sync_read=True,
        )
        # for i in range(255):
        #     try:
        #         self.io.disable_torque([i])
        #         print("i", i)
        #     except:
        #         print("aze")
        #         pass
        # exit()

        self.id = id

        # TODO zero first
        self.io.enable_torque([self.id])
        self.io.set_mode({self.id: 0})
        self.io.set_goal_position({self.id: 0})
        time.sleep(1)
        self.io.disable_torque([self.id])
        # exit()

        self.io.set_mode({self.id: 2})
        if kp is None:
            self.kp = self.io.get_P_coefficient([self.id])[0]
        else:
            self.kp = kp


        self.pwm = 0
        self.control_freq = 100  # Hz
        self.goal_position = 0
        self.present_position = 0
        self.last_position = 0
        self.present_speed = 0
        self.speed_decimation = 2
        self.last_t = time.time()
        Thread(target=self.update, daemon=True).start()

    def disable_torque(self):
        self.io.set_mode({self.id: 0})
        self.io.disable_torque([self.id])

    def enable_torque(self):
        self.io.enable_torque([self.id])
        self.io.set_mode({self.id: 2})

    def get_present_speed(self, rad=True):
        if rad:
            return np.deg2rad(self.present_speed)
        return self.present_speed

    def update(self):
        i = 0
        while True:
            s = time.time()
            self.present_position = self.io.get_present_position([self.id])[0]
            error = self.goal_position - self.present_position

            self.pwm = self.kp * error
            self.pwm = np.int16(self.pwm)
            self.pwm = np.clip(self.pwm, -1000, 1000)

            pwm_magnitude = abs(self.pwm)

            direction_bit = 1 if self.pwm >= 0 else 0
            goal_time = (direction_bit << 10) | pwm_magnitude

            self.io.set_goal_time({self.id: goal_time})

            if i % self.speed_decimation == 0:
                self.present_speed = (self.present_position - self.last_position) / (
                    time.time() - self.last_t
                )
                self.last_position = self.present_position
                self.last_t = time.time()
            took = time.time() - s
            time.sleep(max(0, 1 / self.control_freq - took))
            i += 1


# motor = FeetechPWMControl()
