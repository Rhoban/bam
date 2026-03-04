# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import os
import numpy as np
from dynamixel_sdk import *

# =============================================================================
# Control table – Protocol V1 (MX / AX series)
# =============================================================================
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

# =============================================================================
# Control table – XL-320 (Protocol V2 only)
# =============================================================================
# --- EEPROM ---
XL320_ADDR_CW_ANGLE_LIMIT = 6       # 2 bytes
XL320_ADDR_CCW_ANGLE_LIMIT = 8      # 2 bytes
XL320_ADDR_CONTROL_MODE = 11        # 1 byte  (1=Wheel, 2=Joint)
XL320_ADDR_MAX_TORQUE = 15          # 2 bytes
# --- RAM ---
XL320_ADDR_TORQUE_ENABLE = 24       # 1 byte
XL320_ADDR_LED = 25                 # 1 byte  (RGB bitmask: R=1, G=2, B=4)
XL320_ADDR_D_GAIN = 27              # 1 byte
XL320_ADDR_I_GAIN = 28              # 1 byte
XL320_ADDR_P_GAIN = 29              # 1 byte
XL320_ADDR_GOAL_POSITION = 30       # 2 bytes  (0–1023, 0.29°/step, range 0–300°)
XL320_ADDR_MOVING_SPEED = 32        # 2 bytes  (0–1023 CCW, 1024–2047 CW, 0.111 rpm/step)
XL320_ADDR_TORQUE_LIMIT = 35        # 2 bytes
XL320_ADDR_PRESENT_POSITION = 37    # 2 bytes
XL320_ADDR_PRESENT_SPEED = 39       # 2 bytes
XL320_ADDR_PRESENT_LOAD = 41        # 2 bytes
XL320_ADDR_PRESENT_VOLTAGE = 45     # 1 byte
XL320_ADDR_PRESENT_TEMPERATURE = 46 # 1 byte
XL320_ADDR_MOVING = 49              # 1 byte  (0=stopped, 1=moving)
XL320_ADDR_HW_ERROR_STATUS = 50     # 1 byte

# XL-320 position resolution
XL320_RESOLUTION = 1023             # 10-bit
XL320_RANGE_DEG = 300.0             # degrees
XL320_CENTER = 512                  # raw value for 0 rad (150°)


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


class DynamixelXL320:
    """
    Controller for the XL-320 servo motor.

    The XL-320 uses **Protocol 2.0 only** and has a 10-bit position encoder
    covering a 300° range (0 to 1023 raw, centre = 512 ≡ 0 rad).

    Args:
        port: Serial port, e.g. ``"/dev/ttyUSB0"``.
        id:   DYNAMIXEL ID (default 1).
    """

    def __init__(self, port: str, id: int = 1):
        self.id = id

        result = os.system(f"setserial {port} low_latency")
        if result != 0:
            raise Exception(
                "Failed to set low latency mode "
                "(you can try: sudo apt install setserial)"
            )

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)  # Protocol 2.0 mandatory

        self.portHandler.openPort()
        self.portHandler.setBaudRate(1000000)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _rad_to_raw(position: float) -> int:
        """Convert radians to XL-320 raw position (0–1023, centre = 0 rad)."""
        raw = int(position * (XL320_RESOLUTION / (XL320_RANGE_DEG * np.pi / 180.0)) + XL320_CENTER)
        return int(np.clip(raw, 0, XL320_RESOLUTION))

    @staticmethod
    def _raw_to_rad(raw: int) -> float:
        """Convert XL-320 raw position to radians."""
        return (raw - XL320_CENTER) * (XL320_RANGE_DEG * np.pi / 180.0) / XL320_RESOLUTION

    # ------------------------------------------------------------------
    # Write commands
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool):
        """Enable or disable motor torque."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_p_gain(self, gain: int):
        """Set the proportional (P) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_P_GAIN, gain
        )

    def set_i_gain(self, gain: int):
        """Set the integral (I) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_I_GAIN, gain
        )

    def set_d_gain(self, gain: int):
        """Set the derivative (D) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_D_GAIN, gain
        )

    def set_pid_gains(self, p: int, i: int, d: int):
        """Set P, I and D gains in one call (values 0–254 each)."""
        self.set_p_gain(p)
        self.set_i_gain(i)
        self.set_d_gain(d)

    def set_goal_position(self, position: float):
        """
        Set goal position.

        Args:
            position: Target angle in radians. Acceptable range:
                      ``[-150°, +150°]`` → ``[-2.618, +2.618]`` rad.
        """
        raw = self._rad_to_raw(position)
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_GOAL_POSITION, raw
        )

    def set_moving_speed(self, speed_rpm: float):
        """
        Set moving speed in Joint Mode.

        Args:
            speed_rpm: Speed in RPM (0 = maximum, 114 = approx. max).
                       The sign is ignored; always positive in joint mode.
        """
        raw = int(abs(speed_rpm) / 0.111)
        raw = int(np.clip(raw, 0, 1023))
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_MOVING_SPEED, raw
        )

    def set_led(self, color: int):
        """
        Set the LED colour using a RGB bitmask.

        Args:
            color: Bitmask – Red=1, Green=2, Blue=4 (combinations allowed, 0=off).
        """
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_LED, color & 0x07
        )

    # ------------------------------------------------------------------
    # Read feedback
    # ------------------------------------------------------------------

    def read_data(self) -> dict:
        """
        Read present position, speed, load, input voltage and temperature.

        Returns a dict with keys:
        ``position`` (rad), ``speed`` (rad/s), ``load`` (signed, 0.1 % units),
        ``input_volts`` (V), ``temp`` (°C).

        Note: addresses 37–42 are contiguous; 43–44 are unused; 45–46 are
        voltage and temperature.  Two separate reads are performed.
        """
        # Read position (2 B), speed (2 B), load (2 B) → 6 bytes from addr 37
        data_psl, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_POSITION, 6
        )
        
        # Present position – 10-bit, 0.29°/step, centre = 512
        position_raw = (data_psl[1] << 8) | data_psl[0]
        position = self._raw_to_rad(position_raw)

        # Present speed – 11-bit value, bit 10 = direction (0=CCW, 1=CW)
        # Unit: 0.111 rpm/step
        speed_raw = (data_psl[3] << 8) | data_psl[2]
        if speed_raw > 1023:
            speed = -(speed_raw - 1024)
        else:
            speed = speed_raw
        speed = speed * 0.111 * 2 * np.pi / 60.0  # rad/s

        # Present load – 11-bit, bit 10 = direction, unit 0.1 %
        load_raw = (data_psl[5] << 8) | data_psl[4]
        if load_raw > 1023:
            load = -(load_raw - 1024)
        else:
            load = load_raw

        # Read voltage (1 B) and temperature (1 B) → 2 bytes from addr 45
        data_vt, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_VOLTAGE, 2
        )

        # Voltage: unit is 0.1 V
        volts = data_vt[0] / 10.0

        # Temperature in °C
        temp = data_vt[1]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }
