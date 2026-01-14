import numpy as np
from rustypot import Xl330PyController
import time

c = Xl330PyController("/dev/ttyACM0", baudrate=1000000, timeout=0.01)
ID = 1

c.write_position_p_gain(ID, 400)
c.write_position_i_gain(ID, 0)
c.write_position_d_gain(ID, 0)
c.write_torque_enable(ID, True)
c.write_goal_position(ID, 0)

input()

def convert_load(load):
    # Applied "load"
    load = (data[5] << 8) | data[4]
    if load > 1024:
        load = -(load - 1024)
    return load
            
        

goal_position = np.deg2rad(30)
c.write_goal_position(ID, goal_position)
while True:
    position = c.read_present_position(ID)[0]
    load = convert_load(c.read_present_pwm(ID)[0])
    
    print(position)
    print(pwm)
    print("==")
    
    
    
    time.sleep(0.01)
    



