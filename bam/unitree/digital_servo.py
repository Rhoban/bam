# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Serial driver for the Unitree J288 / S288 digital servos.

Both servos share one protocol and one gear ratio. The servo answers each
20-byte command frame with a 26-byte state frame and sends nothing unprompted.
All protocol fields are rotor-side; this driver converts to and from
output-side SI units.

Protocol reference: https://support.unitree.com/home/en/J288-S288%20Servo/motor_protocol
and https://github.com/unitreerobotics/digital_servo (python/servo_demo.py).

Probe a servo with stop frames only (no motion)::

    python -m bam.unitree.digital_servo --port /dev/cu.usbmodemXXXX --id 0
"""

import argparse
import struct
import time

import numpy as np

#: Rotor turns per output turn.
GEAR_RATIO = 70070.0 / 243.0  # 288.354

# Protocol resolution, output side. The frame carries each field as a rotor-side
# integer: kp in units of 1/1280000 N.m/rad, kd in 1/128000000 N.m.s/rad, torque
# in 1/256000 N.m and speed in 2*pi/2.56 rad/s.
KP_STEP = GEAR_RATIO**2 / 1280000.0  # 0.0650 N.m/rad
KD_STEP = GEAR_RATIO**2 / 128000000.0  # 0.000650 N.m.s/rad
TORQUE_STEP = GEAR_RATIO / 256000.0  # 0.00113 N.m
SPEED_STEP = 2.0 * np.pi / 2.56 / GEAR_RATIO  # 0.00851 rad/s

MODE_STOP = 0
MODE_FOC = 1
MODE_CLEAR_FAULTS = 6
MODE_RESET = 7


def sent_gains(kp: float, kd: float) -> tuple[float, float]:
    """The gains the servo actually receives for requested output-side gains."""
    return (
        min(32767, max(0, round(kp / KP_STEP))) * KP_STEP,
        min(32767, max(0, round(kd / KD_STEP))) * KD_STEP,
    )


def _crc32_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
        table.append(crc & 0xFFFFFFFF)
    return table


_CRC32_TABLE = _crc32_table()


def crc32(data: bytes) -> int:
    """Unitree's CRC32: MSB-first, poly 0x04C11DB7, init 0xFFFFFFFF, no final
    xor, fed one little-endian 32-bit word at a time (high byte first)."""
    assert len(data) % 4 == 0
    crc = 0xFFFFFFFF
    for i in range(0, len(data), 4):
        for b in (data[i + 3], data[i + 2], data[i + 1], data[i]):
            crc = _CRC32_TABLE[(crc >> 24) ^ b] ^ ((crc << 8) & 0xFFFFFFFF)
    return crc


def _clamp_int(x: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, round(x)))


def build_command(
    motor_id: int,
    mode: int,
    torque: float = 0.0,
    speed: float = 0.0,
    position: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    timeout: bool = True,
) -> bytes:
    """Build a 20-byte command frame. Arguments are output-side:
    torque [N.m], speed [rad/s], position [rad], kp [N.m/rad], kd [N.m.s/rad]."""
    mode_byte = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((1 if timeout else 0) << 7)
    tor = _clamp_int(torque / TORQUE_STEP, -32768, 32767)
    spd = _clamp_int(speed / SPEED_STEP, -32768, 32767)
    pos = _clamp_int(position * GEAR_RATIO * 32768.0 / (2 * np.pi), -(2**31), 2**31 - 1)
    k_pos = _clamp_int(kp / KP_STEP, 0, 32767)
    k_spd = _clamp_int(kd / KD_STEP, 0, 32767)

    body = (
        b"\xfe\xee"
        + bytes([mode_byte, 0])
        + struct.pack("<hhihh", tor, spd, pos, k_pos, k_spd)
    )
    return body + struct.pack("<I", crc32(body))


def clear_faults_command(motor_id: int) -> bytes:
    """Fault clear: status 6 with tor_des = -256 raw and every other field 0."""
    body = b"\xfe\xee" + bytes([(motor_id & 0x0F) | (MODE_CLEAR_FAULTS << 4), 0])
    body += struct.pack("<hhihh", -256, 0, 0, 0, 0)
    return body + struct.pack("<I", crc32(body))


def parse_state(frame: bytes) -> dict | None:
    """Parse a 26-byte state frame into output-side SI units, or None if the
    header or CRC is wrong.

    The frame reports the state from before the command it answers was applied.
    ``speed`` is the servo's own filtered estimate (first order, about 17 ms on
    a J288) and ``torque`` its motor torque from the current, before gearbox
    losses. ``output_encoder`` is only reliable at rest.
    """
    if len(frame) != 26 or frame[0] != 0xFC or frame[1] != 0xEE:
        return None
    if struct.unpack("<I", frame[22:26])[0] != crc32(frame[2:22]):
        return None

    mode_byte = frame[2]
    temp, winding, vol, torque, speed, pos, error, out_pos_flags = struct.unpack(
        "<bBBhhiIH", frame[3:20]
    )
    return {
        "id": mode_byte & 0x0F,
        "mode": (mode_byte >> 4) & 0x07,
        "timeout": (mode_byte >> 7) & 0x01,
        "temp": temp,  # housing [°C]
        "winding_temp": winding,  # [°C]
        "input_volts": vol / 2.0,
        "torque": torque * TORQUE_STEP,  # [N.m]
        "speed": speed * SPEED_STEP,  # [rad/s]
        "position": pos * 2 * np.pi / 32768.0 / GEAR_RATIO,  # multi-turn [rad]
        "output_encoder": (out_pos_flags & 0x1FFF) * 2 * np.pi / 8192.0,  # 0..2pi
        "error": error,
        "warning": (out_pos_flags >> 13) & 0x07,
    }


class DigitalServo:
    """One J288 or S288 on a serial port, in request-reply mode."""

    def __init__(
        self,
        port: str,
        motor_id: int = 0,
        baudrate: int = 6_000_000,
        read_timeout: float = 0.02,
    ):
        import serial

        self.motor_id = motor_id
        self.ser = serial.Serial(port, baudrate, timeout=read_timeout)
        self.ser.reset_input_buffer()

    def _transact(self, frame: bytes) -> dict | None:
        self.ser.write(frame)
        reply = self.ser.read(26)
        state = parse_state(reply)
        if state is None and reply:
            # Out of sync: drop whatever is buffered so the next reply lines up.
            self.ser.reset_input_buffer()
        return state

    def command(
        self,
        mode: int,
        torque=0.0,
        speed=0.0,
        position=0.0,
        kp=0.0,
        kd=0.0,
        timeout=True,
    ) -> dict | None:
        return self._transact(
            build_command(self.motor_id, mode, torque, speed, position, kp, kd, timeout)
        )

    def stop(self) -> dict | None:
        return self.command(MODE_STOP)

    def clear_timeout(self) -> dict | None:
        """Stop frame with the timeout bit clear, which resets a latched timeout."""
        return self.command(MODE_STOP, timeout=False)

    def recover(self, duration: float = 0.5) -> dict:
        """Clear a latched timeout and return the servo's state.

        A servo left without frames for about 1 s latches its timeout and ignores
        enable commands until a frame with the timeout bit clear arrives. The
        clear usually shows in the second reply, but has taken ten.
        """
        state = None
        end = time.perf_counter() + duration
        while time.perf_counter() < end:
            state = self.clear_timeout() or state
            if state is not None and not state["timeout"]:
                return state
        if state is None:
            raise RuntimeError("no reply from servo")
        raise RuntimeError("servo timeout state did not clear")

    def clear_faults(self) -> dict | None:
        return self._transact(clear_faults_command(self.motor_id))

    def close(self):
        if self.ser.is_open:
            self.stop()
            self.ser.close()


def _probe():
    parser = argparse.ArgumentParser(
        description="Probe a J288/S288 with stop frames (no motion)."
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--count", type=int, default=500)
    args = parser.parse_args()

    servo = DigitalServo(args.port, args.id)
    ok, t_rt, last = 0, [], None
    for _ in range(args.count):
        t0 = time.perf_counter()
        state = servo.stop()
        t_rt.append(time.perf_counter() - t0)
        if state is not None:
            ok += 1
            last = state
    servo.ser.close()

    t_rt = np.array(t_rt) * 1000
    print(f"{ok}/{args.count} valid replies")
    print(
        f"round trip ms: median {np.median(t_rt):.2f}, "
        f"p95 {np.percentile(t_rt, 95):.2f}, max {t_rt.max():.2f}"
    )
    for k, v in (last or {}).items():
        if k == "error":
            print(f"  {k:15s} {v:#x}")
        elif isinstance(v, float):
            print(f"  {k:15s} {v:.4f}")
        else:
            print(f"  {k:15s} {v}")


if __name__ == "__main__":
    _probe()
