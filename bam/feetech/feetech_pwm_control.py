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

        self.control_freq = 100  # Hz
        self.goal_position = 0
        self.present_position = 0
        Thread(target=self.update, daemon=True).start()

    def disable_torque(self):
        self.io.set_mode({self.id: 0})
        self.io.disable_torque([self.id])

    def enable_torque(self):
        self.io.enable_torque([self.id])
        self.io.set_mode({self.id: 2})

    def update(self):
        while True:
            self.present_position = self.io.get_present_position([self.id])[0]
            error = self.goal_position - self.present_position

            pwm = self.kp * error
            # pwm *= 10
            pwm = np.int16(pwm)

            pwm_magnitude = abs(pwm)
            if pwm_magnitude >= 2**10:  # cap to 10 bits
                pwm_magnitude = (2**10) - 1

            direction_bit = 1 if pwm >= 0 else 0

            goal_time = (direction_bit << 10) | pwm_magnitude

            self.io.set_goal_time({self.id: goal_time})

            time.sleep(1 / self.control_freq)


# motor = FeetechPWMControl()
