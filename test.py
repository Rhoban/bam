from bam.feetech.feetech_pwm_control import FeetechPWMControl
import numpy as np
import time
import pickle

data = {
    "torque_limit": [],
    "max_torque_limit": [],
    "overload_torque": [],
    "protective_torque": [],
    "protection_current": [],
    "present_current": [],
    "present_voltage": [],
}

KP = 32
F = 0.2
A = 90  # deg

update_freq = 50  # Hz

motor = FeetechPWMControl(id=1)
motor.kp = KP
try:
    while True:
        # target = -90
        target = A * np.sin(2 * np.pi * F * time.time())

        # print("torque limit", motor.io.get_torque_limit([motor.id]))
        # print("max torque limit", motor.io.get_max_torque_limit([motor.id]))
        # print("overload torque", motor.io.get_overload_torque([motor.id]))
        # print("protective torque", motor.io.get_protective_torque([motor.id]))
        # print("protection current", motor.io.get_protection_current([motor.id]))
        # print(
        #     "present current", motor.io.get_present_current([motor.id])[0] * 6.5 * 0.001
        # )
        # print("present voltage", motor.io.get_present_voltage([motor.id])[0] * 0.1)
        load = motor.io.get_present_load([motor.id])[0]
        # print("{0:011b}".format(load))
        # # print(load)
        # # load = (0 << 10) | load
        load = load & ~(1 << 10)
        # print("{0:011b}".format(load))
        print("current load", load)
        print("pwm", motor.pwm)
        # print("==")
        data["torque_limit"].append(motor.io.get_torque_limit([motor.id])[0])
        data["max_torque_limit"].append(motor.io.get_max_torque_limit([motor.id])[0])
        data["overload_torque"].append(motor.io.get_overload_torque([motor.id])[0])
        data["protective_torque"].append(motor.io.get_protective_torque([motor.id])[0])
        data["protection_current"].append(
            motor.io.get_protection_current([motor.id])[0]
        )
        data["present_current"].append(
            motor.io.get_present_current([motor.id])[0] * 6.5 * 0.001
        )
        data["present_voltage"].append(
            motor.io.get_present_voltage([motor.id])[0] * 0.1
        )

        motor.goal_position = target

        time.sleep(1 / update_freq)
except KeyboardInterrupt:
    motor.disable_torque()
    print("Torque disabled")

pickle.dump(data, open("data.pkl", "wb"))
