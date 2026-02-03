import numpy as np
from rustypot import Xl330PyController
import time
import struct

c = Xl330PyController("/dev/ttyACM0", baudrate=1000000, timeout=0.01)
ID = 33

fw_kp = 200

c.write_position_p_gain(ID, fw_kp)
c.write_position_i_gain(ID, 0)
c.write_position_d_gain(ID, 0)
c.write_torque_enable(ID, True)
c.write_goal_position(ID, 0)

input()


ENCODER_COUNTS_PER_REV = 4096
KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)
PWM_LIMIT = 885   # Default Present PWM limit (counts) per eManual
_RADS_PER_SEC_PER_COUNT = 0.229 * (2.0 * np.pi / 60.0)

def convert_velocity(raw_signed: float) -> float:
    return float(raw_signed) * _RADS_PER_SEC_PER_COUNT

def convert_pwm_to_duty(raw: float) -> float:
    x = float(raw)
    
    if x > 2**15-1:
        x -= 2**16
        
    y = np.clip(x / PWM_LIMIT, -1.0, 1.0)
    
    return y

            
        

goal_position = np.deg2rad(30)
c.write_goal_position(ID, goal_position)
while True:
    position = c.read_present_position(ID)[0]
    duty = convert_pwm_to_duty(c.read_present_pwm(ID)[0])
    speed = convert_velocity(c.read_present_velocity(ID)[0])
       
    error = goal_position - position
    # print(error)
    # print(load)
    # print("==")
    
    print("error", error)
    print("duty", duty)
    # duty = error_gain * error * fw_kp
    # duty/error_gain = error * fw_kp
    if error != 0:
        error_gain = duty / (error * fw_kp)
        print("error gain", error_gain)
        # Expected theoretical error_gain given divisor/PWM_LIMIT
        expected_error_gain = (ENCODER_COUNTS_PER_REV / (2 * np.pi)) / (KP_DIVISOR * PWM_LIMIT)
        print("expected error gain", expected_error_gain)
    else:
        print("error gain: N/A (zero error)")
    print("==")
    
    
    
    time.sleep(0.01)
    

