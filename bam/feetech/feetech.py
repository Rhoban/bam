import os
import numpy as np
from openlch import hal as olch_hal

# Torque enable
ADDR_TORQUE_ENABLE = 24
# P Gain
ADDR_P_GAIN = 28
# Goal position
ADDR_GOAL_POSITION = 30
# Present position (2 bytes)
ADDR_PRESENT_POSITION = 36
# Present speed (2 bytes)
ADDR_PRESENT_SPEED = 38
# Present load (2 bytes)
ADDR_PRESENT_LOAD = 40
# Present voltage (1 byte)
ADDR_PRESENT_VOLTAGE = 42
# Present temperature (1 byte)
ADDR_PRESENT_TEMPERATURE = 43


class FeetechSTS3250:
    def __init__(self, ip: str, id: int = 1):
        self.id = id
        self.hal = olch_hal.HAL(ip)
        self.servo = self.hal.servo

    def set_p_gain(self, gain: int):
        pass
        # Set P gain
        
        ## unimplemented

    def set_torque(self, enable: bool):
        pass
        # Enable torque
        ## unimplemented, torque is always enabled

    def set_goal_position(self, position: float):
        # convert to degrees
        position = position * 180.0 / np.pi
        self.servo.set_positions([(self.id, position)])

    def read_data(self):
        # Reading position, speed, load, voltage and temperature

        data = self.servo.get_servo_info(self.id)

        # Position is in degrees (convert to radians)
        position = data["current_position"] / 180.0 * np.pi

        # Speed is degrees per second (convert to rpm)
        speed = data["speed"] / 6.0

        # Applied "load"
        load = 0  # unimplemented

        # Voltage is a byte value, units are 0.1 V
        volts = data["voltage"]

        # Temperature are °C
        temp = data["temperature"]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }
