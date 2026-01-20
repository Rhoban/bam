import numpy as np
from rustypot import Xl330PyController
import time
import struct

c = Xl330PyController("/dev/ttyACM0", baudrate=1000000, timeout=0.01)
ID = 1

fw_kp = 400

c.write_position_p_gain(ID, fw_kp)
c.write_position_i_gain(ID, 0)
c.write_position_d_gain(ID, 0)
c.write_torque_enable(ID, True)
c.write_goal_position(ID, 0)

input()


_RADS_PER_SEC_PER_COUNT = 0.229 * (2.0 * np.pi / 60.0)

def convert_velocity(raw_signed: float) -> float:
    return float(raw_signed) * _RADS_PER_SEC_PER_COUNT

def convert_load(raw: float) -> float:
    x = float(raw)
    
    if x > 2**15-1:
        x -= 2**16
        
    y = np.clip(x / 1023.0, -1.0, 1.0)
    
    return y

            
        

goal_position = np.deg2rad(30)
c.write_goal_position(ID, goal_position)
while True:
    position = c.read_present_position(ID)[0]
    load = convert_load(c.read_present_pwm(ID)[0])
    speed = convert_velocity(c.read_present_velocity(ID)[0])
       
    error = goal_position - position
    # print(error)
    # print(load)
    # print("==")
    
    
    # load = error_gain * error * fw_kp
    # load/error_gain = error * fw_kp
    error_gain = load / (error * fw_kp)
    print(error_gain)
    
    
    
    time.sleep(0.01)
    



