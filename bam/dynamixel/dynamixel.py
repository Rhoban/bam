import os
import numpy as np
from dynamixel_sdk import *

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


class DynamixelActuatorV1:
    def __init__(self, port: str, id: int = 1):
        self.id = id

        result = os.system(f"setserial {port} low_latency")
        if result != 0:
            raise Exception("Failed to set low latency mode (you can try: sudo apt install setserial)")

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(1.0)

        self.portHandler.openPort()
        self.portHandler.setBaudRate(1000000)

    def set_p_gain(self, gain: int):
        # Set P gain
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_P_GAIN, gain
        )

    def set_torque(self, enable: bool):
        # Enable torque
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_goal_position(self, position: float):
        # Position is a 12-bit value
        position = int(4096 * (position / (2 * np.pi) + 0.5))

        # Set goal position
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_GOAL_POSITION, position
        )

    def read_data(self):
        # Reading position, speed, load, voltage and temperature
        data, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, ADDR_PRESENT_POSITION, 8
        )

        # Position is a 12-bit value
        position = (data[1] << 8) | data[0]
        position = 2 * np.pi * ((position / 4096) - 0.5)

        # Speed is a 10-bit value, units are 0.11 rpm per step
        speed = (data[3] << 8) | data[2]
        if speed > 1024:
            speed = -(speed - 1024)
        speed = speed * 0.11 * 2 * np.pi / 60.0

        # Applied "load"
        load = (data[5] << 8) | data[4]
        if load > 1024:
            load = -(load - 1024)

        # Voltage is a byte value, units are 0.1 V
        volts = data[6] / 10.0

        # Temperature are °C
        temp = data[7]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }
