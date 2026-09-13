# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");
#
# Hardware driver for Feetech HLS / SMS series serial bus servos with magnetic encoder
# over USB-to-half-duplex UART converter board.
# Standalone module: does NOT depend on BAM core.

from __future__ import annotations
import math
import time
from typing import Optional, Dict, Any, List, Tuple

try:
    import serial
except ImportError:
    serial = None


class FeetechHLSDriver:
    """
    Hardware driver for Feetech HLS serial bus servos (magnetic encoder version)
    using FT-SCS / SMS protocol over half-duplex UART.
    """

    # Instruction definitions
    INST_PING = 0x01
    INST_READ_DATA = 0x02
    INST_WRITE_DATA = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_RECOVERY = 0x06
    INST_RESET = 0x0A
    INST_POS_CALIBRATION = 0x0B
    INST_SYNC_READ = 0x82
    INST_SYNC_WRITE = 0x83

    # Key Memory Table Addresses
    ADDR_ID = 0x05
    ADDR_BAUDRATE = 0x06
    ADDR_MODE = 0x21          # 0: Pos, 1: Speed, 2: Constant Current, 3: Open-loop PWM
    ADDR_TORQUE_ENABLE = 0x28  # 0: Off, 1: On, 2: Damping
    ADDR_ACCELERATION = 0x29
    ADDR_GOAL_POSITION = 0x2A  # 2 bytes
    ADDR_GOAL_CURRENT = 0x2C   # modes 0-2: current; mode 3: PWM (see set_pwm)
    ADDR_GOAL_SPEED = 0x2E     # 2 bytes (running speed limit)
    ADDR_TORQUE_LIMIT = 0x30   # 2 bytes
    ADDR_KP = 0x32
    ADDR_KD = 0x33
    ADDR_KI = 0x34
    ADDR_LOCK = 0x37           # 0: Save to EEPROM on write, 1: RAM only

    # Feedback Addresses
    ADDR_PRESENT_POSITION = 0x38     # 2 bytes, 0.087 deg/LSB (4096 / rev)
    ADDR_PRESENT_SPEED = 0x3A        # 2 bytes, 0.732 RPM/LSB
    ADDR_PRESENT_LOAD = 0x3C         # 2 bytes, 0.1 % duty cycle
    ADDR_PRESENT_VOLTAGE = 0x3E      # 1 byte, 0.1 V
    ADDR_PRESENT_TEMPERATURE = 0x3F  # 1 byte, deg C
    ADDR_SERVO_STATUS = 0x41         # 1 byte (error status bits)
    ADDR_PRESENT_CURRENT = 0x45      # 2 bytes, 6.5 mA/LSB, Bit15 sign

    # Unit scales
    POSITION_RAD_PER_LSB = (2.0 * math.pi) / 4096.0
    SPEED_RAD_S_PER_LSB = 0.732 * (2.0 * math.pi / 60.0)  # 0.732 RPM -> rad/s

    # Sign conventions differ per register, as the memory table specifies:
    CURRENT_SIGN_BIT = 0x8000  # 0x2C (modes 0-2) and 0x45: Bit15 = direction
    PWM_SIGN_BIT = 0x0400      # 0x2C (mode 3) and 0x3C: Bit10 = direction

    # The HLS memory table quotes 6.5 mA/LSB for the current registers (0x1C /
    # 0x2C / 0x45), but that table is a family-wide document and the HLS2915
    # does not use it: measured on hardware (100% duty, stalled, bench-supply
    # ammeter reading the phase current directly) 408 register steps drew
    # 0.64 A, i.e. 1.569 mA/LSB — the table
    # value is 4.14x too coarse. Cross-checks that confirm it:
    #
    #   * winding resistance 4.8 V / 0.64 A = 7.56 ohm, versus 7.06 ohm from an
    #     independent 12 V / 22.7%-duty point and 8.0 ohm implied by the
    #     datasheet's 1.5 A stall rating;
    #   * the 11.0 kg.cm peak then falls at 0.98 A, giving Kt = 0.97 N.m/A,
    #     against 0.93 from the datasheet stall point and ~1.0 from its no-load
    #     speed (with Kt == Ke in SI units).
    #
    # Identification is insensitive to this constant (the recorder's command and
    # the model's control law share one unit), but every ampere reported outside
    # BAM is not: with this value the commanded amperes are real amperes, and
    # the model's kt is a physical torque constant (~0.97 N.m/A).
    CURRENT_AMPS_PER_LSB = 0.001569

    def __init__(
        self,
        port: Optional[str] = "/dev/ttyUSB0",
        baudrate: int = 1000000,
        timeout: float = 0.02,
        has_echo: bool = False,
        retries: int = 2,
    ):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.has_echo = has_echo
        self.retries = retries
        self.ser: Optional[serial.Serial] = None

        if port is not None:
            self.open()

    def open(self):
        if serial is None:
            raise ImportError("pyserial is required: pip install pyserial")
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    @staticmethod
    def calc_checksum(servo_id: int, length: int, instruction: int, params: List[int]) -> int:
        s = servo_id + length + instruction + sum(params)
        return (~s) & 0xFF

    def _send_packet(self, servo_id: int, instruction: int, params: List[int]) -> None:
        length = len(params) + 2
        chk = self.calc_checksum(servo_id, length, instruction, params)
        pkt = bytearray([0xFF, 0xFF, servo_id, length, instruction] + params + [chk])
        self.ser.write(pkt)

        if self.has_echo:
            # Discard local echo from half-duplex transceiver
            self.ser.read(len(pkt))

    def _read_packet(self, expected_id: int) -> Tuple[int, List[int]]:
        """
        Reads and validates a response packet:
        0xFF 0xFF ID Length Error [Params...] CheckSum
        """
        start_t = time.time()
        head = bytearray()
        while time.time() - start_t < self.timeout:
            b = self.ser.read(1)
            if not b:
                continue
            head.append(b[0])
            if len(head) >= 2 and head[-2] == 0xFF and head[-1] == 0xFF:
                break

        if len(head) < 2 or head[-2] != 0xFF or head[-1] != 0xFF:
            raise TimeoutError(f"Timeout waiting for packet header (0xFF 0xFF) from ID {expected_id}")

        hdr = self.ser.read(3)
        if len(hdr) < 3:
            raise TimeoutError("Timeout reading packet header fields")

        ret_id, length, error = hdr[0], hdr[1], hdr[2]
        param_len = length - 2
        payload = self.ser.read(param_len + 1)
        if len(payload) < param_len + 1:
            raise TimeoutError("Timeout reading response payload")

        params = list(payload[:param_len])
        chk = payload[param_len]

        calc_chk = self.calc_checksum(ret_id, length, error, params)
        if calc_chk != chk:
            raise ValueError(f"Checksum mismatch: received {chk:#02x}, expected {calc_chk:#02x}")

        return error, params

    def ping(self, servo_id: int = 1) -> bool:
        """Query servo online status."""
        try:
            self._send_packet(servo_id, self.INST_PING, [])
            err, _ = self._read_packet(servo_id)
            return err == 0
        except Exception:
            return False

    def read_bytes(self, servo_id: int, addr: int, length: int) -> List[int]:
        """Read arbitrary registers from memory table.

        Retried once on a timeout/checksum error: the half-duplex bus picks up
        occasional glitches from motor switching, and a single dropped byte
        inside a 6 s recording should not abort the run.
        """
        last = None
        for attempt in range(self.retries + 1):
            try:
                self._send_packet(servo_id, self.INST_READ_DATA, [addr, length])
                err, data = self._read_packet(servo_id)
                if err != 0:
                    raise RuntimeError(f"Servo ID {servo_id} reported error status: {err:#02x}")
                return data
            except (TimeoutError, ValueError) as exc:
                last = exc
                if attempt < self.retries:
                    time.sleep(0.005)
        raise last

    def write_bytes(self, servo_id: int, addr: int, data: List[int]) -> int:
        """Write arbitrary registers to memory table (retried like read_bytes)."""
        params = [addr] + list(data)
        if servo_id == 0xFE:  # Broadcast, no reply
            self._send_packet(servo_id, self.INST_WRITE_DATA, params)
            return 0
        last = None
        for attempt in range(self.retries + 1):
            try:
                self._send_packet(servo_id, self.INST_WRITE_DATA, params)
                err, _ = self._read_packet(servo_id)
                return err
            except (TimeoutError, ValueError) as exc:
                last = exc
                if attempt < self.retries:
                    time.sleep(0.005)
        raise last

    def set_mode(self, servo_id: int, mode: int, max_speed_limit: int = 3000) -> int:
        """
        Set operating mode (Address 0x21):
        0: Position mode
        1: Speed mode
        2: Constant current mode
        3: PWM mode
        """
        res = self.write_bytes(servo_id, self.ADDR_MODE, [mode & 0xFF])
        if mode in (1, 2, 3):
            # Speed limit (0x2E) must be non-zero to allow motion in current/speed modes
            spd_bytes = [max_speed_limit & 0xFF, (max_speed_limit >> 8) & 0xFF]
            self.write_bytes(servo_id, self.ADDR_GOAL_SPEED, spd_bytes)
        return res

    def set_torque_enable(self, servo_id: int, enable: bool) -> int:
        """Enable (1) or disable (0) torque output."""
        val = 1 if enable else 0
        return self.write_bytes(servo_id, self.ADDR_TORQUE_ENABLE, [val])

    def set_goal_current(self, servo_id: int, current_amps: float) -> int:
        """
        Set target current (A) in constant current mode (Address 0x2C).
        Scale: 6.5 mA/LSB, range +/- 2047 steps.
        Bit 15 is direction: 1 for positive joint angle (+), 0 for negative (-).
        """
        steps = int(round(abs(current_amps) / self.CURRENT_AMPS_PER_LSB))
        steps = min(steps, 2047)
        raw_val = (steps | self.CURRENT_SIGN_BIT) if current_amps >= 0 else steps
        data = [raw_val & 0xFF, (raw_val >> 8) & 0xFF]
        return self.write_bytes(servo_id, self.ADDR_GOAL_CURRENT, data)

    def set_pwm(self, servo_id: int, duty: float) -> int:
        """
        Set the open-loop PWM duty in PWM mode (mode 3, Address 0x2C).

        Address 0x2C is shared with the goal-current register, and the memory
        table redefines it per mode: in modes 0-2 it is a current (Bit 15 sign,
        6.5 mA/LSB), in mode 3 a PWM value in -1000..1000 with **Bit 10** as the
        direction bit. Never call this outside mode 3 — 1000 | 0x0400 read as a
        current is a large negative command, not a duty.

        At duty 1000 the bridge is fully on and the chopping factor disappears,
        which is what makes a stalled 100%-duty run measurable with a plain
        supply ammeter.

        :param duty: Duty in per-mille of full scale, -1000..1000 (1000 = 100%).
        """
        magnitude = min(int(round(abs(duty))), 1000)
        raw_val = (magnitude | self.PWM_SIGN_BIT) if duty < 0 else magnitude
        data = [raw_val & 0xFF, (raw_val >> 8) & 0xFF]
        return self.write_bytes(servo_id, self.ADDR_GOAL_CURRENT, data)

    def read_current_raw(self, servo_id: int = 1) -> int:
        """
        Read the present-current register (0x45) as a signed step count.

        Unlike :meth:`read_status` this returns the raw register value, with no
        assumption about the LSB scale, so it can be compared against a directly
        measured current to work out that scale.

        :returns: Signed current in register steps (Bit 15 is the direction bit).
        """
        raw = self.read_bytes(servo_id, self.ADDR_PRESENT_CURRENT, 2)
        value = raw[0] | (raw[1] << 8)
        magnitude = value & 0x7FFF
        return -magnitude if (value & self.CURRENT_SIGN_BIT) else magnitude

    def calibrate_position(self, servo_id: int = 1, value: Optional[int] = 0) -> int:
        """
        Command 0x0B: declare the *current* pose to be a given encoder value.

        This is how the servo's own zero acquires a physical meaning, and it is
        what makes recording without a per-run software offset possible — the
        convention BAM's other recorders follow. It matters here: gearbox
        stiction lets the arm rest anywhere in a band (measured 13 degrees wide
        on this rig), so "wherever the arm happens to be at startup" is not a
        usable zero reference.

        Per the protocol document, no parameter means "current pose becomes the
        middle position"; with a value, the current pose becomes that value. The
        result is stored in EEPROM, so unlock first (0x37 = 0) to keep it across
        power cycles.

        :param value: Encoder value to assign to the current pose (default 0,
            i.e. make the current pose the zero). ``None`` selects the
            middle-position form of the instruction.
        :returns: The servo's error byte.
        """
        params = [] if value is None else [value & 0xFF, (value >> 8) & 0xFF]
        self._send_packet(servo_id, self.INST_POS_CALIBRATION, params)
        err, _ = self._read_packet(servo_id)
        return err

    def read_status(self, servo_id: int = 1) -> Dict[str, Any]:
        """
        Single-packet read of 15 contiguous feedback registers starting at 0x38.
        Returns normalized physical units (rad, rad/s, A, V, deg C).
        """
        raw = self.read_bytes(servo_id, self.ADDR_PRESENT_POSITION, 15)

        # Position (0x38, 2 bytes), Bit15 = direction
        raw_pos = raw[0] | (raw[1] << 8)
        pos_sign = -1.0 if (raw_pos & self.CURRENT_SIGN_BIT) else 1.0
        pos_rad = (raw_pos & 0x7FFF) * pos_sign * self.POSITION_RAD_PER_LSB

        # Speed (0x3A, 2 bytes), Bit15 = direction
        raw_spd = raw[2] | (raw[3] << 8)
        spd_sign = -1.0 if (raw_spd & self.CURRENT_SIGN_BIT) else 1.0
        spd_rad_s = (raw_spd & 0x7FFF) * spd_sign * self.SPEED_RAD_S_PER_LSB

        # Load / drive duty (0x3C, 2 bytes), 0.1 %/LSB, Bit10 = direction
        raw_load = raw[4] | (raw[5] << 8)
        load_sign = -1.0 if (raw_load & self.PWM_SIGN_BIT) else 1.0
        load_pct = (raw_load & 0x03FF) * load_sign * 0.1

        # Bus voltage (0x3E, 1 byte, 0.1 V) and temperature (0x3F, 1 byte, °C)
        volts = raw[6] * 0.1
        temp = raw[7]

        # Error status (0x41, 1 byte)
        status_err = raw[9]

        # Present current (0x45, 2 bytes), Bit15 = direction
        raw_cur = raw[13] | (raw[14] << 8)
        cur_sign = -1.0 if (raw_cur & self.CURRENT_SIGN_BIT) else 1.0
        cur_amps = (raw_cur & 0x7FFF) * cur_sign * self.CURRENT_AMPS_PER_LSB

        return {
            "position": pos_rad,
            "speed": spd_rad_s,
            "load": load_pct,
            "input_volts": volts,
            "temp": temp,
            "status": status_err,
            "current": cur_amps,
        }
