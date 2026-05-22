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
ADDR_TORQUE_ENABLE       = 24
ADDR_P_GAIN              = 28
ADDR_GOAL_POSITION       = 30
ADDR_PRESENT_POSITION    = 36
ADDR_PRESENT_SPEED       = 38
ADDR_PRESENT_LOAD        = 40
ADDR_PRESENT_VOLTAGE     = 42
ADDR_PRESENT_TEMPERATURE = 43

# =============================================================================
# Control table – XL-320 (Protocol V2 only)
# =============================================================================
XL320_ADDR_CW_ANGLE_LIMIT      = 6
XL320_ADDR_CCW_ANGLE_LIMIT     = 8
XL320_ADDR_CONTROL_MODE        = 11
XL320_ADDR_MAX_TORQUE          = 15
XL320_ADDR_TORQUE_ENABLE       = 24
XL320_ADDR_LED                 = 25
XL320_ADDR_D_GAIN              = 27
XL320_ADDR_I_GAIN              = 28
XL320_ADDR_P_GAIN              = 29
XL320_ADDR_GOAL_POSITION       = 30   # 2 bytes (0–1023)
XL320_ADDR_MOVING_SPEED        = 32
XL320_ADDR_TORQUE_LIMIT        = 35
XL320_ADDR_PRESENT_POSITION    = 37
XL320_ADDR_PRESENT_SPEED       = 39
XL320_ADDR_PRESENT_LOAD        = 41
XL320_ADDR_PRESENT_VOLTAGE     = 45
XL320_ADDR_PRESENT_TEMPERATURE = 46
XL320_ADDR_MOVING              = 49
XL320_ADDR_HW_ERROR_STATUS     = 50

XL320_RESOLUTION = 1023
XL320_RANGE_DEG  = 300.0
XL320_CENTER     = 512

# =============================================================================
# Control table – XM430-W350 (Protocol V2 only)
# =============================================================================
# EEPROM / common
XM430_ADDR_OPERATING_MODE        = 11   # 1 byte, default 3 = Position Control Mode
XM430_ADDR_PROTOCOL_TYPE         = 13   # 1 byte, default 2
XM430_ADDR_PWM_LIMIT             = 36   # 2 bytes
XM430_ADDR_CURRENT_LIMIT         = 38   # 2 bytes
# RAM
XM430_ADDR_TORQUE_ENABLE         = 64   # 1 byte
XM430_ADDR_POSITION_D_GAIN       = 80   # 2 bytes
XM430_ADDR_POSITION_I_GAIN       = 82   # 2 bytes
XM430_ADDR_POSITION_P_GAIN       = 84   # 2 bytes
XM430_ADDR_FEEDFORWARD_2ND_GAIN  = 88   # 2 bytes
XM430_ADDR_FEEDFORWARD_1ST_GAIN  = 90   # 2 bytes
XM430_ADDR_BUS_WATCHDOG          = 98   # 1 byte
XM430_ADDR_GOAL_PWM              = 100  # 2 bytes
XM430_ADDR_GOAL_CURRENT          = 102  # 2 bytes
XM430_ADDR_GOAL_VELOCITY         = 104  # 4 bytes
XM430_ADDR_PROFILE_ACCELERATION  = 108  # 4 bytes
XM430_ADDR_PROFILE_VELOCITY      = 112  # 4 bytes
XM430_ADDR_GOAL_POSITION         = 116  # 4 bytes
XM430_ADDR_PRESENT_PWM           = 124  # 2 bytes
XM430_ADDR_PRESENT_CURRENT       = 126  # 2 bytes, signed, unit 2.69 mA/step
XM430_ADDR_PRESENT_VELOCITY      = 128  # 4 bytes, signed, unit 0.229 rpm/step
XM430_ADDR_PRESENT_POSITION      = 132  # 4 bytes
XM430_ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, unit 0.1 V
XM430_ADDR_PRESENT_TEMPERATURE   = 146  # 1 byte

XM430_RESOLUTION = 4096
XM430_CENTER     = 2048


def _set_low_latency(port: str):
    """Try setserial low_latency; warn but do not abort if unavailable."""
    result = os.system(f"setserial {port} low_latency 2>/dev/null")
    if result != 0:
        print(
            f"WARNING: setserial low_latency failed on {port}. "
            "Communication may still work. Install setserial for lower latency."
        )


class DynamixelActuatorV1:
    def __init__(self, port: str, id: int = 1, baudrate: int = 1000000):
        self.id = id
        _set_low_latency(port)

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(1.0)

        self.portHandler.openPort()
        self.portHandler.setBaudRate(baudrate)

    def set_p_gain(self, gain: int):
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_P_GAIN, gain
        )

    def set_torque(self, enable: bool):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_goal_position(self, position: float):
        position = int(4096 * (position / (2 * np.pi) + 0.5))
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_GOAL_POSITION, position
        )

    def read_data(self):
        data, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, ADDR_PRESENT_POSITION, 8
        )

        position = (data[1] << 8) | data[0]
        position = 2 * np.pi * ((position / 4096) - 0.5)

        speed = (data[3] << 8) | data[2]
        if speed > 1024:
            speed = -(speed - 1024)
        speed = speed * 0.11 * 2 * np.pi / 60.0

        load = (data[5] << 8) | data[4]
        if load > 1024:
            load = -(load - 1024)

        volts = data[6] / 10.0
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
    Controller for the XL-320 servo motor (Protocol 2.0, 10-bit position over 300°).
    """

    def __init__(self, port: str, id: int = 1, baudrate: int = 1000000):
        self.id = id
        _set_low_latency(port)

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)

        self.portHandler.openPort()
        self.portHandler.setBaudRate(baudrate)

    @staticmethod
    def _rad_to_raw(position: float) -> int:
        raw = int(position * (XL320_RESOLUTION / (XL320_RANGE_DEG * np.pi / 180.0)) + XL320_CENTER)
        return int(np.clip(raw, 0, XL320_RESOLUTION))

    @staticmethod
    def _raw_to_rad(raw: int) -> float:
        return (raw - XL320_CENTER) * (XL320_RANGE_DEG * np.pi / 180.0) / XL320_RESOLUTION

    def set_torque(self, enable: bool):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_p_gain(self, gain: int):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_P_GAIN, gain
        )

    def set_i_gain(self, gain: int):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_I_GAIN, gain
        )

    def set_d_gain(self, gain: int):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_D_GAIN, gain
        )

    def set_pid_gains(self, p: int, i: int, d: int):
        self.set_p_gain(p)
        self.set_i_gain(i)
        self.set_d_gain(d)

    def set_goal_position(self, position: float):
        raw = self._rad_to_raw(position)
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_GOAL_POSITION, raw
        )

    def set_moving_speed(self, speed_rpm: float):
        raw = int(np.clip(int(abs(speed_rpm) / 0.111), 0, 1023))
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_MOVING_SPEED, raw
        )

    def set_led(self, color: int):
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_LED, color & 0x07
        )

    def read_data(self) -> dict:
        data_psl, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_POSITION, 6
        )

        position_raw = (data_psl[1] << 8) | data_psl[0]
        position = self._raw_to_rad(position_raw)

        speed_raw = (data_psl[3] << 8) | data_psl[2]
        if speed_raw > 1023:
            speed = -(speed_raw - 1024)
        else:
            speed = speed_raw
        speed = speed * 0.111 * 2 * np.pi / 60.0

        load_raw = (data_psl[5] << 8) | data_psl[4]
        if load_raw > 1023:
            load = -(load_raw - 1024)
        else:
            load = load_raw

        data_vt, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_VOLTAGE, 2
        )
        volts = data_vt[0] / 10.0
        temp = data_vt[1]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }


class DynamixelXM430W350:
    """
    Controller for DYNAMIXEL XM430-W350-T/R (Protocol 2.0).

    Position convention:
      q = 0 rad  → raw 2048 (180°, motor center)
      q = -π rad → raw ≈ 0
      q = +π rad → raw ≈ 4095
    Align the pendulum mechanically so raw 2048 is the downward zero position.
    """

    def __init__(self, port: str, id: int = 1, baudrate: int = 57600,
                 low_latency: bool = True):
        self.id = id
        self.port = port
        self.baudrate = baudrate

        if low_latency:
            _set_low_latency(port)

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        if not self.portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set baudrate {baudrate}")

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_signed(value: int, bits: int) -> int:
        sign_bit = 1 << (bits - 1)
        mask = (1 << bits) - 1
        value &= mask
        return value - (1 << bits) if value & sign_bit else value

    @staticmethod
    def _u16(data, i: int) -> int:
        return data[i] | (data[i + 1] << 8)

    @staticmethod
    def _u32(data, i: int) -> int:
        return data[i] | (data[i + 1] << 8) | (data[i + 2] << 16) | (data[i + 3] << 24)

    @classmethod
    def _s16(cls, data, i: int) -> int:
        return cls._to_signed(cls._u16(data, i), 16)

    @classmethod
    def _s32(cls, data, i: int) -> int:
        return cls._to_signed(cls._u32(data, i), 32)

    def _check_result(self, result, error, context: str):
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"{context} (id={self.id}): "
                f"{self.packetHandler.getTxRxResult(result)} — "
                f"check baudrate({self.baudrate})/cable/ID"
            )
        if error != 0:
            raise RuntimeError(
                f"{context} (id={self.id}): "
                f"{self.packetHandler.getRxPacketError(error)}"
            )

    # ------------------------------------------------------------------
    # Register write helpers (TxRx = confirmed write)
    # ------------------------------------------------------------------

    def write1(self, addr: int, value: int, context: str = "write1"):
        result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.id, addr, value
        )
        self._check_result(result, error, context)

    def write2(self, addr: int, value: int, context: str = "write2"):
        result, error = self.packetHandler.write2ByteTxRx(
            self.portHandler, self.id, addr, value & 0xFFFF
        )
        self._check_result(result, error, context)

    def write4(self, addr: int, value: int, context: str = "write4"):
        result, error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.id, addr, value & 0xFFFFFFFF
        )
        self._check_result(result, error, context)

    # ------------------------------------------------------------------
    # Position conversion
    # ------------------------------------------------------------------

    @staticmethod
    def _rad_to_raw(position: float) -> int:
        raw = int(round(XM430_RESOLUTION * (position / (2 * np.pi) + 0.5)))
        return int(np.clip(raw, 0, XM430_RESOLUTION - 1))

    @staticmethod
    def _raw_to_rad(raw: int) -> float:
        raw = raw % XM430_RESOLUTION
        return 2 * np.pi * ((raw / XM430_RESOLUTION) - 0.5)

    # ------------------------------------------------------------------
    # Configuration commands
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool):
        self.write1(XM430_ADDR_TORQUE_ENABLE, 1 if enable else 0, "set_torque")

    def set_operating_mode(self, mode: int = 3):
        """Must be called while torque is disabled. 3 = Position Control Mode."""
        self.write1(XM430_ADDR_OPERATING_MODE, mode, "set_operating_mode")

    def set_p_gain(self, gain: int):
        """Position P Gain, range 0–16383, default 800."""
        self.write2(XM430_ADDR_POSITION_P_GAIN, int(np.clip(gain, 0, 16383)), "set_p_gain")

    def set_i_gain(self, gain: int):
        self.write2(XM430_ADDR_POSITION_I_GAIN, int(np.clip(gain, 0, 16383)), "set_i_gain")

    def set_d_gain(self, gain: int):
        self.write2(XM430_ADDR_POSITION_D_GAIN, int(np.clip(gain, 0, 16383)), "set_d_gain")

    def set_pid_gains(self, p: int, i: int = 0, d: int = 0):
        self.set_d_gain(d)
        self.set_i_gain(i)
        self.set_p_gain(p)

    def set_profile(self, velocity: int = 0, acceleration: int = 0):
        """0 disables profile shaping (fastest step response)."""
        self.write4(XM430_ADDR_PROFILE_ACCELERATION, acceleration, "set_profile_accel")
        self.write4(XM430_ADDR_PROFILE_VELOCITY, velocity, "set_profile_vel")

    def set_bus_watchdog(self, value: int = 0):
        self.write1(XM430_ADDR_BUS_WATCHDOG, value, "set_bus_watchdog")

    def prepare_for_recording(self, kp: int):
        """Safe one-call setup before a BAM position trajectory recording session."""
        self.set_torque(False)
        self.set_operating_mode(3)      # Position Control Mode
        self.set_bus_watchdog(0)
        self.set_profile(velocity=0, acceleration=0)
        self.set_pid_gains(p=kp, i=0, d=0)

    # ------------------------------------------------------------------
    # Motion command
    # ------------------------------------------------------------------

    def set_goal_position(self, position: float):
        raw = self._rad_to_raw(position)
        self.write4(XM430_ADDR_GOAL_POSITION, raw, "set_goal_position")

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------

    def read_data(self) -> dict:
        """
        Bulk read from Present PWM (124) through Present Temperature (146).
        23 bytes total, no gaps in that range.

        Offsets within the 23-byte payload:
          0-1:  Present PWM       (s16)
          2-3:  Present Current   (s16, unit 2.69 mA/step)
          4-7:  Present Velocity  (s32, unit 0.229 rpm/step)
          8-11: Present Position  (s32)
         12-15: Velocity Trajectory (skip)
         16-19: Position Trajectory (skip)
         20-21: Present Input Voltage (u16, unit 0.1 V)
         22:    Present Temperature  (u8, °C)
        """
        data, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XM430_ADDR_PRESENT_PWM, 23
        )
        self._check_result(result, error, "read_data")

        pwm_raw      = self._s16(data, 0)
        current_raw  = self._s16(data, 2)
        velocity_raw = self._s32(data, 4)
        position_raw = self._s32(data, 8)
        voltage_raw  = self._u16(data, 20)
        temp         = data[22]

        position      = self._raw_to_rad(position_raw)
        speed         = velocity_raw * 0.229 * 2 * np.pi / 60.0   # rad/s
        current_amps  = current_raw * 0.00269                      # A
        pwm_percent   = pwm_raw * 0.113                            # %
        volts         = voltage_raw / 10.0                         # V

        return {
            "position":      position,
            "speed":         speed,
            # BAM logs expect a "load" key; XM430 exposes current instead
            "load":          current_raw,
            "input_volts":   volts,
            "temp":          temp,
            "current":       current_amps,
            "current_raw":   current_raw,
            "pwm":           pwm_percent,
            "pwm_raw":       pwm_raw,
            "position_raw":  position_raw,
            "velocity_raw":  velocity_raw,
        }

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self):
        try:
            self.set_torque(False)
        except Exception as e:
            print(f"WARNING: failed to torque off during close: {e}")
        try:
            self.portHandler.closePort()
        except Exception:
            pass
