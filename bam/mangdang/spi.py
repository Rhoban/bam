# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Host driver for the MangDang MD-300-150 through an ESP32-S3 SPI bridge.

Each MD-300-150 module has an AT32 microcontroller that drives its three
outputs and speaks SPI. An ESP32-S3 running the bridge firmware (to be
published) exposes up to four modules on a
USB serial port. This module implements the host half of the bridge protocol::

    request   10 bytes: 42 cmd servo 00 arg1:u16 arg2:u16 crc:u16
    response  14 bytes: 62 cmd servo status pos_dd:u16 cur_mA:i16 val:f32 crc:u16

Everything on the wire is little-endian and CRC16-CCITT-FALSE is computed over
every byte but the trailing CRC itself. Servo ids run from 1 to 12 and module
``(id - 1) // 3`` owns each servo.

Units: the AT32 reports positions in **deci-degrees** (0.1 deg per count) over
``0 .. scale_dd``. The scale must equal ``range_position_deg * 10`` as configured
on the AT32 (default ``3100`` = 310.0 deg) and its centre is the middle of the
potentiometer travel. :class:`SpiMD300IO` exposes that raw world, while its
BAM-shaped methods convert to radians with ``0.0`` at the centre of travel.

Firmware: the AT32 is expected to run firmware 0.0.9. There, control_mode 1 turns the PID output into duty with ``pwm_scale``
(param 12), the live ids are 0..12, and the reported current is the
supply-side current (see :func:`motor_current_ma`).

Instantiating with ``dry_run=True`` returns a driver that answers every command
with plausible synthetic data and never touches a serial port, so the recorder
can be exercised without hardware (see ``record.py --dry-run``).
"""

from __future__ import annotations

import math
import struct
import time

MAGIC_REQ = 0x42
MAGIC_RSP = 0x62

CMD_PING = 0
CMD_SET_POS = 1
CMD_IDLE = 2
CMD_TORQUE = 3
CMD_SET_KP = 4
CMD_SET_KD = 5
CMD_GET_PARAM = 6
CMD_GET_LIVE = 7
CMD_SET_SCALE = 8
CMD_RAW_REPLY = 9
CMD_SET_PARAM = 10
CMD_GET_STATUS = 11

#: Commands the bridge carries out with AT32 config frames. The AT32 loads its
#: config reply into the frame clocked out next, so the first ordinary frame of
#: that board afterwards carries the reply in the board's first channel instead
#: of its feedback (position = the parameter id, ``res`` = the value). The
#: other two channels are unaffected.
CONFIG_COMMANDS = frozenset(
    (CMD_SET_KP, CMD_SET_KD, CMD_GET_PARAM, CMD_GET_LIVE, CMD_SET_PARAM)
)
#: Commands whose reply carries a board's feedback.
FEEDBACK_COMMANDS = frozenset(
    (CMD_PING, CMD_SET_POS, CMD_IDLE, CMD_TORQUE, CMD_GET_STATUS)
)

BAM_OK = 0

#: Human-readable status codes returned by the firmware.
STATUS = {
    0: "OK",
    1: "ERR_SERVO",
    2: "ERR_ARG",
    3: "ERR_BOARD",
    4: "ERR_CRC",
    5: "ERR_CMD",
}

#: AT32 ``sms_config`` parameter ids (mirrors ``driver_board.h``).
PARAM = {
    "reverse_position_sensor": 0,
    "min_position_adc": 1,
    "max_position_adc": 2,
    "range_position_deg": 3,
    "reverse_motor": 4,
    "kp_position": 5,
    "kd_position": 6,
    "kp_current": 7,
    "kff_current": 8,
    "max_pwm_duty_cycle": 9,
    "ki_position": 10,  # position integral gain [mA / (deg*s)], 0 = off
    "control_mode": 11,  # 0 = legacy P+FF current loop, 1 = PID -> PWM (voltage mode)
    # Duty per unit of PID output in control_mode 1 (``duty = pwm_scale * PID``;
    # default 0.001, so 1000 = 100 %).
    "pwm_scale": 12,
}
#: ``control_mode`` values (AT32 ``sms_config.control_mode``).
CONTROL_MODE_LEGACY = 0
CONTROL_MODE_PID_PWM = 1
PARAM_BY_ID = {v: k for k, v in PARAM.items()}

#: AT32 live control-loop value ids (mirrors ``driver_board.h``).
LIVE = {
    "pos_adc": 0,
    "cur_adc": 1,
    "setpoint_pos_deg": 2,
    "present_pos_deg": 3,
    "error_pos_deg": 4,
    "max_current_mA": 5,
    "setpoint_cur_mA": 6,
    "present_cur_mA": 7,
    "error_cur_mA": 8,
    "pwm_duty": 9,
    "mode": 10,
    "loop_counter": 11,
    "temperature_c": 12,  # housing NTC [degC]
}
LIVE_BY_ID = {v: k for k, v in LIVE.items()}
#: Live ids the AT32 knows (0..12). It does not reject a larger id: it answers
#: live 0 (the position ADC) instead, so :meth:`SpiMD300IO.get_live` refuses
#: ids outside this range.
LIVE_COUNT = 13

#: ``present_cur_mA`` (the feedback current and live 7) is the **supply-side**
#: current: the mid-pulse shunt reading times the PWM duty, ``I_motor * |duty|``.
#: The motor current is therefore ``present_cur_mA / |duty|`` (see
#: :func:`motor_current_ma`).
#: Below this |duty| the division amplifies noise without bound; the motor
#: current is computed with ``max(|duty|, MIN_DUTY_FOR_CURRENT)`` instead.
MIN_DUTY_FOR_CURRENT = 0.05


def motor_current_ma(reported_ma: float, duty: float) -> float:
    """Motor current [mA] from a supply-side reading and the PWM duty.

    ``reported_ma`` already carries the sign of the duty, so dividing by
    ``|duty|`` keeps it. Below :data:`MIN_DUTY_FOR_CURRENT` the divisor is held
    at that floor: the result is then a lower bound, not a measurement.
    """
    return float(reported_ma) / max(abs(float(duty)), MIN_DUTY_FOR_CURRENT)


DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

#: ``range_position_deg * 10`` on the AT32 (310.0 deg). Kept in sync with the
#: board by :meth:`SpiMD300IO.sync_scale_from_board`.
DEFAULT_SCALE_DD = 3100.0

#: Current cap [mA] used by the position commands. In position mode the AT32
#: reads the torque field as a *cap*, not a forced draw.
DEFAULT_CUR_MA = 900

#: Servo ids addressable through the bridge (4 boards x 3 channels).
SERVO_IDS = tuple(range(1, 13))

#: Nominal supply voltage. The module does not measure it, so this is a constant.
DEFAULT_VIN = 12.0


def module_servos(servo: int) -> tuple[int, int, int]:
    """The three servo ids on the same module as ``servo``."""
    first = (servo - 1) // 3 * 3 + 1
    return (first, first + 1, first + 2)


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, init ``0xFFFF``, poly ``0x1021``, MSB-first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            )
    return crc


def pack_float(value: float) -> tuple[int, int]:
    """Pack a float into the two 16-bit argument words the firmware expects."""
    return struct.unpack("<HH", struct.pack("<f", value))


class BridgeError(IOError):
    """Raised when the firmware answers with a non-OK status."""


def pick_port(port: str | None) -> str:
    """Return ``port`` if given, else auto-detect a single USB-serial port.

    USB bridges (CH340K, CP210x, FTDI) are preferred over the virtual ports that
    sometimes clutter ``/dev`` under Linux.

    :param port: Explicit device path, or ``None`` to auto-detect.
    :returns: The device path to open.
    :raises SystemExit: If no port, or several candidate ports, are found.
    """
    if port:
        return port
    try:
        from serial.tools import list_ports
    except ImportError:
        raise SystemExit("pyserial is not installed. Run: pip install pyserial")

    ports = list(list_ports.comports())
    usb = [
        p
        for p in ports
        if (p.vid is not None)
        or "USB" in (p.device or "").upper()
        or "USB" in (p.description or "").upper()
    ]
    candidates = usb or ports

    if not candidates:
        raise SystemExit(
            "No serial port found. Is the bridge plugged in? "
            "Pass --port explicitly, e.g. --port /dev/ttyUSB0"
        )
    if len(candidates) > 1:
        print("Several serial ports found:")
        for p in candidates:
            print(f"  {p.device}  ({p.description})")
        raise SystemExit(
            f"Re-run with --port <one of the above>, e.g. --port {candidates[0].device}"
        )
    return candidates[0].device


class SpiMD300IO:
    """Host-side driver for one or more MD-300-150 modules behind the SPI bridge.

    The object is a thin, stateful wrapper around the serial link. It caches the
    last commanded goal per servo so that :meth:`enable_torque` can re-command
    the same goal (which is how the firmware re-enters position mode), and it
    keeps the previous sample to derive a velocity by differentiation.

    :param port: Serial device path (e.g. ``/dev/ttyUSB0``).
    :param baud: Bridge baud rate.
    :param timeout: Per-response timeout [s].
    :param scale_dd: Full-scale travel in deci-degrees. Must equal
        ``range_position_deg * 10`` on the AT32 boards. Call
        :meth:`sync_scale_from_board` once connected to adopt the board value.
    :param dry_run: If ``True``, never open the serial port and synthesise
        responses instead (used for offline testing of the recorder).
    :param zero_dd: Raw deci-degree reading of the pendulum's vertical. The
        BAM-shaped methods map ``0 rad`` here instead of the pot centre, so an
        arm that cannot be mounted exactly at the centre is still logged
        around its true zero. ``None`` means the pot centre (``scale_dd / 2``).
    :param cur_ma: Current cap [mA] sent with every position command.
    """

    def __init__(
        self,
        port: str,
        baud: int = 921600,
        timeout: float = 1.0,
        scale_dd: float = DEFAULT_SCALE_DD,
        dry_run: bool = False,
        zero_dd: float | None = None,
        cur_ma: int = DEFAULT_CUR_MA,
    ):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.scale_dd = float(scale_dd)
        self.dry_run = dry_run
        self._zero_dd = None if zero_dd is None else float(zero_dd)
        self.cur_ma = int(cur_ma)
        #: Raw `res` feedback word of the last feedback response (temperature).
        self.last_res = 0

        #: Last commanded deci-degrees per servo, used by enable_torque.
        self._goal_dd: dict[int, int] = {}
        #: Last (time, pos_dd) sample, used by read_present_velocity.
        self._prev_t = 0.0
        self._prev_dd: int | None = None
        self._velocity = 0.0
        #: Boards (0..3) whose next feedback frame is not feedback: the first
        #: frame after a port open is stale, and the first after a config
        #: command carries the config reply in the board's first channel.
        self._stale_boards: set[int] = set(range(4))

        self.ser = None
        if not dry_run:
            import serial  # imported lazily so dry-run needs no pyserial

            # Exclusive: a second tool opening the port would reset the ESP32
            # and interleave frames with this one.
            self.ser = serial.Serial(port, baud, timeout=timeout, exclusive=True)
            time.sleep(0.2)  # let the firmware finish booting
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    # ---- lifecycle -------------------------------------------------------
    def close(self) -> None:
        """Release the serial port. Safe to call more than once."""
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self) -> SpiMD300IO:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- transport -------------------------------------------------------
    def _request(
        self, cmd: int, servo: int = 0, arg1: int = 0, arg2: int = 0, pad: int = 0
    ) -> dict:
        """Send one command and return its response, with valid feedback.

        Before a feedback command on a board whose next frame is known not to
        carry feedback (see :data:`CONFIG_COMMANDS`), one ``PING`` is sent and
        its reply discarded, so no caller ever reads a config reply as a
        position.
        """
        board = (servo - 1) // 3 if 1 <= servo <= 12 else None
        if cmd in FEEDBACK_COMMANDS and board in self._stale_boards:
            self._send(CMD_PING, servo)
            self._stale_boards.discard(board)
        response = self._send(cmd, servo, arg1, arg2, pad)
        if cmd in CONFIG_COMMANDS and board is not None:
            self._stale_boards.add(board)
        return response

    def _send(
        self, cmd: int, servo: int = 0, arg1: int = 0, arg2: int = 0, pad: int = 0
    ) -> dict:
        """Send one command frame and return the decoded response.

        A boot banner byte or a lost frame can desync the stream once, so the
        frame is written a second time before giving up; resyncing from a fresh
        write always recovers.

        :raises TimeoutError: If no response arrives.
        :raises BridgeError: If the firmware answers with a non-OK status.
        """
        if self.dry_run:
            return self._dry_response(cmd, servo, arg1, arg2, pad)

        head = struct.pack(
            "<BBBBHH", MAGIC_REQ, cmd, servo, pad & 0xFF, arg1 & 0xFFFF, arg2 & 0xFFFF
        )
        frame = head + struct.pack("<H", crc16(head))

        last_err: Exception | None = None
        for _ in range(2):
            self.ser.write(frame)
            try:
                return self._check(self._read_response())
            except (TimeoutError, OSError) as exc:
                last_err = exc
                self.ser.reset_input_buffer()
        raise last_err  # type: ignore[misc]

    def _read_response(self) -> dict:
        """Read and validate one 14-byte response frame."""
        buf = bytearray()
        deadline = time.monotonic() + self.timeout

        # Skip the firmware's human-readable "# ..." boot lines.
        while time.monotonic() < deadline:
            byte = self.ser.read(1)
            if not byte:
                continue
            if byte[0] == MAGIC_RSP:
                buf += byte
                break
            if byte[0] == ord("#"):
                while time.monotonic() < deadline:
                    c = self.ser.read(1)
                    if c in (b"", b"\n"):
                        break

        if not buf:
            raise TimeoutError("no response (wrong port? wrong firmware?)")

        while len(buf) < 14 and time.monotonic() < deadline:
            buf += self.ser.read(14 - len(buf))
        if len(buf) < 14:
            raise TimeoutError(f"short response: {len(buf)} bytes")

        raw = bytes(buf[:14])
        _magic, cmd, servo, status, pos_dd, cur_ma, val, crc = struct.unpack(
            "<BBBBHhfH", raw
        )
        if crc16(raw[:12]) != crc:
            raise OSError("response CRC mismatch")
        return {
            "cmd": cmd,
            "servo": servo,
            "status": status,
            "pos_dd": pos_dd,
            "cur_mA": cur_ma,
            "val": val,
        }

    @staticmethod
    def _check(response: dict) -> dict:
        """Raise :class:`BridgeError` when the firmware reported a failure."""
        if response["status"] != BAM_OK:
            raise BridgeError(
                f"firmware error: {STATUS.get(response['status'], response['status'])}"
            )
        return response

    # ---- raw firmware operations (mirror the C command set) --------------
    def ping(self, servo: int) -> dict:
        """Refresh one servo's feedback without changing its setpoint."""
        response = self._request(CMD_PING, servo)
        self._note_feedback(response)
        return response

    def _note_feedback(self, response: dict) -> None:
        """Keep the raw ``res`` word the feedback commands return in ``val``."""
        self.last_res = struct.unpack("<I", struct.pack("<f", response["val"]))[0]

    def set_pos_dd(self, servo: int, pos_dd: int, cur_ma: int | None = None) -> dict:
        """Enter position mode and command a raw deci-degree setpoint.

        :param pos_dd: Raw AT32 position, clamped to ``0 .. scale_dd``.
        :param cur_ma: Maximum current the firmware may apply [mA]; defaults
            to the driver's ``cur_ma``.
        :returns: The response, which carries the feedback taken in the same
            SPI transaction (position and current after the previous command).
        """
        pos_dd = max(0, min(round(self.scale_dd), round(pos_dd)))
        self._goal_dd[servo] = pos_dd
        response = self._request(
            CMD_SET_POS, servo, pos_dd, self.cur_ma if cur_ma is None else cur_ma
        )
        self._note_feedback(response)
        return response

    def set_idle(self, servo: int) -> dict:
        """Motor off: the AT32 enters idle mode and the output goes free."""
        return self._request(CMD_IDLE, servo)

    def set_torque_ma(self, servo: int, cur_ma: int) -> dict:
        """Raw current mode, signed mA (spin tests / identification)."""
        return self._request(CMD_TORQUE, servo, cur_ma & 0xFFFF)

    def set_p_gain(self, servo: int, kp: float) -> dict:
        """Write the firmware ``kp_position``."""
        lo, hi = pack_float(float(kp))
        return self._request(CMD_SET_KP, servo, lo, hi)

    def set_d_gain(self, servo: int, kd: float) -> dict:
        """Write the firmware ``kd_position``."""
        lo, hi = pack_float(float(kd))
        return self._request(CMD_SET_KD, servo, lo, hi)

    def get_param(self, servo: int, param_id: int) -> dict:
        """Read one ``sms_config`` parameter from the AT32."""
        return self._request(CMD_GET_PARAM, servo, param_id)

    def get_live(self, servo: int, live_id: int) -> dict:
        """Read one live control-loop value from the AT32 (ids 0..LIVE_COUNT-1)."""
        if not 0 <= int(live_id) < LIVE_COUNT:
            # The AT32 would answer live 0 (the position ADC) without an error.
            raise ValueError(f"live id {live_id} not in 0..{LIVE_COUNT - 1}")
        return self._request(CMD_GET_LIVE, servo, live_id)

    def set_param(self, servo: int, param_id: int, value: float) -> float:
        """Write one ``sms_config`` parameter (RAM only) and return the readback.

        Any of the parameters in :data:`PARAM` can be written, including the
        current-loop gains the recorder must control (``kp_current``,
        ``kff_current``, ``max_pwm_duty_cycle``), ``ki_position``,
        ``control_mode`` and ``pwm_scale``. Nothing is saved to the AT32 flash.
        """
        lo, hi = pack_float(float(value))
        return float(self._request(CMD_SET_PARAM, servo, lo, hi, pad=param_id)["val"])

    def get_status(self, servo: int) -> int:
        """The raw 16-bit status word of the servo's feedback frame."""
        return int(self._request(CMD_GET_STATUS, servo)["val"])

    def dump_params(self, servo: int) -> dict[str, float]:
        """Read all ``sms_config`` parameters, keyed by name."""
        return {
            name: float(self.get_param(servo, pid)["val"])
            for name, pid in PARAM.items()
        }

    def dump_live(self, servo: int) -> dict[str, float]:
        """Read all live control-loop values, keyed by name."""
        return {
            name: float(self.get_live(servo, lid)["val"]) for name, lid in LIVE.items()
        }

    def set_scale_dd(self, scale_dd: float) -> float:
        """Set the host-side and firmware-side full-scale travel [deci-deg]."""
        response = self._request(CMD_SET_SCALE, 0, int(scale_dd))
        self.scale_dd = float(response["val"])
        return self.scale_dd

    def raw_reply(self, servo: int) -> dict:
        """Diagnostic: byte statistics of the 36 raw MISO bytes.

        ``ff_count == 36`` means no board is driving MISO (line pulled up, i.e.
        board absent or unpowered); a varying count means the line is floating.
        """
        response = self._request(CMD_RAW_REPLY, servo)
        packed = struct.unpack("<I", struct.pack("<f", response["val"]))[0]
        return {
            "ff_count": packed & 0xFF,
            "b0": (packed >> 8) & 0xFF,
            "b1": (packed >> 16) & 0xFF,
            "b2": (packed >> 24) & 0xFF,
        }

    def sync_scale_from_board(self, servo: int) -> float:
        """Adopt the AT32 ``range_position_deg`` as the host full-scale."""
        reading = self.get_param(servo, PARAM["range_position_deg"])
        return self.set_scale_dd(reading["val"] * 10.0)

    # ---- unit helpers ----------------------------------------------------
    @property
    def zero_dd(self) -> float:
        """Raw deci-degree reading that the BAM-shaped methods call ``0 rad``."""
        return self.scale_dd / 2.0 if self._zero_dd is None else self._zero_dd

    @zero_dd.setter
    def zero_dd(self, value: float | None) -> None:
        self._zero_dd = None if value is None else float(value)

    def rad_to_dd(self, rad: float) -> int:
        """Convert a joint angle [rad] to raw deci-degrees.

        ``0.0 rad`` maps to :attr:`zero_dd` (the pot centre unless a measured
        vertical was given).
        """
        return round(rad * RAD2DEG * 10.0 + self.zero_dd)

    def dd_to_rad(self, dd: float) -> float:
        """Convert raw deci-degrees to a joint angle [rad], zero at :attr:`zero_dd`."""
        return (dd - self.zero_dd) * 0.1 * DEG2RAD

    def dd_to_deg(self, dd: float) -> float:
        """Convert raw deci-degrees to degrees, zero at :attr:`zero_dd`."""
        return (dd - self.zero_dd) * 0.1

    # ---- BAM-driver-shaped API (what record.py calls) --------------------
    def set_mode(self, mode: dict) -> None:
        """Mirror the Feetech driver's ``set_mode``.

        The bridge has no explicit mode setting: the mode is implied by the
        frame that is sent (position / torque / idle). Only position mode (the
        BAM recorder's mode) is accepted here; anything else is rejected so a
        silent mismatch cannot slip through.

        :param mode: Mapping ``{servo_id: mode}``.
        """
        for value in mode.values():
            if value not in (1, "position"):
                raise ValueError(
                    "SpiMD300IO.set_mode only supports position mode (1); the "
                    "bridge infers idle/torque from the command that is sent"
                )
        self._mode_set = True

    def enable_torque(self, ids) -> None:
        """Re-command each servo's last goal, which powers the motor."""
        for servo in ids:
            goal = self._goal_dd.get(servo, int(self.scale_dd / 2))
            self.set_pos_dd(servo, goal)

    def disable_torque(self, ids) -> None:
        """Enter idle mode on each servo (motor off, output free)."""
        for servo in ids:
            self.set_idle(servo)

    def set_goal_position(self, goal: dict) -> None:
        """Command joint angles.

        :param goal: Mapping ``{servo_id: angle}``. Angles are radians unless
            :meth:`set_goal_position_unit` selected degrees.
        """
        for servo, angle in goal.items():
            radians = angle if self._goal_unit == "rad" else angle * DEG2RAD
            self.set_pos_dd(servo, self.rad_to_dd(radians))

    def set_goal_position_unit(self, unit: str) -> None:
        """Select the unit of :meth:`set_goal_position` (``"rad"`` or ``"deg"``).

        .. note::

            BAM records everything in radians, so the default is ``"rad"``. The
            Feetech driver speaks degrees; pass ``"deg"`` when porting code that
            follows that convention.
        """
        if unit not in ("rad", "deg"):
            raise ValueError("unit must be 'rad' or 'deg'")
        self._goal_unit = unit

    def set_P_coefficient(self, gain: dict) -> None:
        """Write ``kp_position`` on each servo.

        :param gain: Mapping ``{servo_id: kp}``.
        """
        for servo, kp in gain.items():
            self.set_p_gain(servo, kp)

    def set_D_coefficient(self, gain: dict) -> None:
        """Write ``kd_position`` on each servo.

        :param gain: Mapping ``{servo_id: kd}``.
        """
        for servo, kd in gain.items():
            self.set_d_gain(servo, kd)

    def get_present_position(self, ids) -> list[float]:
        """Read joint angles [rad] (``0.0`` at the centre of travel)."""
        return [self.dd_to_rad(self.ping(servo)["pos_dd"]) for servo in ids]

    def get_present_position_dd(self, ids) -> list[int]:
        """Read raw deci-degrees, close to what the AT32 reports."""
        return [int(self.ping(servo)["pos_dd"]) for servo in ids]

    def get_present_speed(self, ids) -> list[float]:
        """Read velocities [rad/s], differentiated on the host.

        The AT32 does not report velocity, so this is a finite difference of the
        position feedback. Unlike :meth:`read_present_velocity` the raw
        difference is returned (no smoothing): the recorder applies its own.
        """
        velocities = []
        for servo in ids:
            response = self.ping(servo)
            t = time.monotonic()
            dd = response["pos_dd"]
            if self._prev_dd is not None and t > self._prev_t:
                velocities.append(
                    (dd - self._prev_dd) * 0.1 * DEG2RAD / (t - self._prev_t)
                )
                self._prev_dd, self._prev_t = dd, t
            else:
                self._prev_dd, self._prev_t = dd, t
                velocities.append(0.0)
        return velocities

    def get_present_current(self, ids) -> list[int]:
        """Read the signed currents [mA] as the AT32 reports them.

        That is the supply-side current; see :func:`motor_current_ma` for the
        motor current."""
        return [int(self.ping(servo)["cur_mA"]) for servo in ids]

    def read_sample(self, servo: int, response: dict | None = None) -> dict:
        """Read position, current and velocity in a single SPI transaction.

        Every firmware response already carries both the position and the
        current, so calling :meth:`get_present_position`,
        :meth:`get_present_current` and :meth:`get_present_speed` separately
        would issue three redundant round-trips. This method issues one, and
        differentiates the position against the previous call to obtain the
        velocity. The recorder uses it on its hot path.

        :param servo: Servo id (1..12).
        :param response: A response already obtained from a feedback command
            (``SET_POS``/``PING``/``IDLE``); when given no extra round trip is
            made, which is how the recorder samples while torque is on.
        :returns: Dict with ``pos_dd`` (raw), ``position`` [rad],
            ``current_mA``, ``speed`` [rad/s] and ``t`` (monotonic timestamp
            taken as soon as the response was decoded).
        """
        if response is None:
            response = self.ping(servo)
        t = time.monotonic()
        dd = response["pos_dd"]

        speed = 0.0
        if self._prev_dd is not None and t > self._prev_t:
            speed = (dd - self._prev_dd) * 0.1 * DEG2RAD / (t - self._prev_t)
        self._prev_dd, self._prev_t = dd, t

        return {
            "pos_dd": int(dd),
            "position": self.dd_to_rad(dd),
            "current_mA": int(response["cur_mA"]),  # as reported (supply side)
            "speed": speed,
            "t": t,
        }

    def get_present_voltage(self, ids) -> list[float]:
        """Read supply voltages [V].

        The module does not measure its supply, so the nominal 12 V is
        reported.
        """
        return [DEFAULT_VIN for _ in ids]

    def get_present_temperature(self, ids) -> list[float]:
        """Temperature is not exposed by the board; kept for API symmetry."""
        return [0.0 for _ in ids]

    def get_present_pwm_duty(self, ids) -> list[float]:
        """Read the applied PWM duty cycle in ``[-1, 1]``.

        One ``DB_LIVE_PWM_DUTY`` transaction per servo (slower than a ping).
        """
        return [float(self.get_live(servo, LIVE["pwm_duty"])["val"]) for servo in ids]

    def read_present_velocity(self, servo: int, alpha: float = 0.2) -> float:
        """Velocity [rad/s] via host-side differentiation, exponentially smoothed.

        :param alpha: Per-sample EMA gain (``0.2`` is a mild low-pass).
        """
        response = self.ping(servo)
        t = time.monotonic()
        dd = response["pos_dd"]
        if self._prev_dd is not None and t > self._prev_t:
            raw = (dd - self._prev_dd) * 0.1 * DEG2RAD / (t - self._prev_t)
            self._velocity += alpha * (raw - self._velocity)
        self._prev_dd, self._prev_t = dd, t
        return self._velocity

    def scan(self) -> list[int]:
        """Probe every servo id and return those whose board answers."""
        found = []
        for servo in SERVO_IDS:
            try:
                self.ping(servo)
            except (OSError, TimeoutError):
                continue
            found.append(servo)
        return found

    # ---- synthetic responses (dry-run only) ------------------------------
    def _dry_response(
        self, cmd: int, servo: int, arg1: int, arg2: int, pad: int = 0
    ) -> dict:
        """Fabricate a response for offline testing; never touches hardware.

        A position command is echoed back as the measured position, so the
        recorder sees a step response rather than a static readout, and written
        parameters are remembered so ``get_param`` round-trips like the real
        board.
        """
        if not hasattr(self, "_dry_state"):
            self._dry_state: dict = {"goals": {}, "params": {}}

        goals = self._dry_state["goals"]
        params = self._dry_state["params"]
        pos_dd = goals.setdefault(servo, int(self.scale_dd / 2))
        value = 0.0

        if cmd == CMD_SET_POS:
            pos_dd = max(0, min(int(self.scale_dd), arg1))
            goals[servo] = pos_dd
        elif cmd in (CMD_SET_KP, CMD_SET_KD):
            bits = arg1 | (arg2 << 16)
            stored = struct.unpack("<f", struct.pack("<I", bits))[0]
            params[
                (servo, PARAM["kp_position" if cmd == CMD_SET_KP else "kd_position"])
            ] = stored
            value = stored
        elif cmd == CMD_SET_PARAM:
            bits = arg1 | (arg2 << 16)
            stored = struct.unpack("<f", struct.pack("<I", bits))[0]
            params[(servo, pad)] = stored
            value = stored
        elif cmd == CMD_GET_PARAM:
            value = params.get((servo, arg1), 0.0)
        elif cmd in (CMD_GET_LIVE, CMD_GET_STATUS):
            value = 0.0

        return {
            "cmd": cmd,
            "servo": servo,
            "status": BAM_OK,
            "pos_dd": pos_dd,
            "cur_mA": 0,
            "val": value,
        }


# ``_goal_unit`` is set here rather than in ``__init__`` so subclasses that
# override ``__init__`` still get a sane default.
SpiMD300IO._goal_unit = "rad"
