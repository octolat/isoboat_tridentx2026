"""Effector base class and concrete effectors (Thruster, Servo).

Effector writes are buffered: each `set_*` call updates the port's pending
setpoint in the state mirror. The TX loop in `_link` flushes the full command
frame at a fixed rate (keepalive on idle).

A value outside the student-facing API range is CLAMPED to that range, not
rejected: mixing two joysticks routinely overshoots, and a robot that keeps
driving at full duty is better teaching than one that stops with a traceback
mid-match. The brain still owns final hardware protection and may clamp
internally.
"""

from __future__ import annotations

import math

from ._codec import encode_port_word
from .enums import Tag


I13_MIN = -4096
I13_MAX = 4095

THRUSTER_DUTY_MIN = -100
THRUSTER_DUTY_MAX = 100
SERVO_ANGLE_MIN = 0.0
SERVO_ANGLE_MAX = 180.0
SERVO_ANGLE_CENTRE = 90.0
SERVO_MICROS_MIN = 500
SERVO_MICROS_MAX = 2500
SERVO_SPEED_MIN = 100
SERVO_SPEED_MAX = 360
SERVO_SPEED_DEFAULT = 180


def _clamp(value: float, low: float, high: float) -> float:
    """Bring `value` inside [low, high]. Anything past an end sits at that end."""
    return max(low, min(high, value))


def _require_i13(value: int, label: str) -> int:
    """Guard the 13-bit command field.

    Unreachable from student input now that every public setter clamps first,
    and kept for that reason: `encode_port_word` MASKS rather than saturates, so
    a value that slipped through would wrap to the opposite sign instead of
    pinning at the limit. This turns a library bug into an exception rather than
    a thruster that reverses.
    """
    value = int(value)
    if not I13_MIN <= value <= I13_MAX:
        raise ValueError(f"{label}={value} cannot be encoded in the 13-bit MARLIN command field.")
    return value


class Effector:
    """What every motor and servo has in common.

    You never make one of these yourself. You name the type in `configure()`
    (`effectors={"M1": Thruster}`) and MARLIN builds the object for you; you
    get it back with `robot.thruster("M1")`, `robot.servo("M5")` or
    `robot.effector("M1")`.

    Attributes:
        port: the port it is plugged into, e.g. "M1".
        type: its type name in lower case, e.g. "thruster".
    """

    def __init__(self, port: str, state_mirror=None) -> None:
        """Bind this effector to a port and the shared state mirror."""
        self.port: str = port
        self.type: str = self.__class__.__name__.lower()
        self._state = state_mirror
        self._warned_not_finite: set[str] = set()

    def _commandable(self, value: float, setter: str) -> bool:
        """False only for nan, the one input no clamp can place.

        Infinities are fine: they are past an end, and clamping pins them there
        like any other overshoot. nan compares False against everything, so
        `max/min` would quietly hand back a limit instead - a nan duty would read
        as full throttle. Leave the setpoint alone instead, and say so once per
        setter so a 50 Hz loop does not bury the console.
        """
        if not math.isnan(value):
            return True
        if setter not in self._warned_not_finite:
            self._warned_not_finite.add(setter)
            print(
                f"[API] {self.port}: {setter}({value}) is not a usable number, so the "
                f"setpoint is left as it was. Check the maths feeding it."
            )
        return False

    def _buffer(self, tag: Tag, value: int) -> None:
        if self._state is None:
            raise RuntimeError(f"{self.port}: effector is not attached to a robot state mirror.")
        self._state.buffer_setpoint(self.port, encode_port_word(tag, _require_i13(value, tag.name)))


class Thruster(Effector):
    """An underwater thruster - a motor with a propeller. One command:
    `set_duty()`.

    You tell it how hard to push, as a percentage. It has no idea how fast it
    is actually spinning and nothing measures that, so the same duty pushes
    differently depending on the battery, the water and the propeller. Getting
    from "how hard" to "how far the robot went" is your job.

    There is no current or power reading yet: the robot reports current as 0
    until next season's firmware update, so those properties are commented out
    below rather than always handing back zero.
    """

    def set_duty(self, duty: float) -> None:
        """Set how hard the thruster pushes: -100 to 100.

        100 is full forward, -100 is full reverse, and 0 stops it. In between
        is proportional power, not speed - 50 is not half speed.

        The thruster keeps pushing at whatever you last gave it until you set
        something else, so a robot that should coast needs `set_duty(0)`.

        Anything past either end is pinned to that end rather than raising, so
        mixing two joysticks is safe to write directly:

            robot.thruster("M1").set_duty(forward + turn)   # 150 pushes as 100
        """
        value = float(duty)
        if not self._commandable(value, "set_duty"):
            return
        value = _clamp(value, THRUSTER_DUTY_MIN, THRUSTER_DUTY_MAX)
        # The hardware runs the other way round: a positive duty on the wire
        # drives the thruster backwards. Invert here so the student-facing sign
        # is the intuitive one, and the inversion lives in exactly one place.
        # The range is symmetric, so negating can never overflow the 13-bit field.
        self._buffer(Tag.THRUSTER_DUTY, -int(round(value)))

    # Disabled pending the firmware current-sensing update next season. The wire
    # format is unchanged (current_a still arrives, pinned at 0); restore both
    # properties as-is once the brain reports a real reading.
    #
    # @property
    # def current(self) -> float:
    #     """Telemetry readback of measured thruster current (A)."""
    #     status = self._state.get_status() if self._state is not None else None
    #     telemetry = (status or {}).get("effectors", {}).get(self.port, {})
    #     return float(telemetry.get("current_a", telemetry.get("current", 0.0)))
    #
    # @property
    # def power(self) -> float:
    #     """Telemetry readback of V*I power (W). Read-only."""
    #     status = self._state.get_status() if self._state is not None else None
    #     telemetry = (status or {}).get("effectors", {}).get(self.port, {})
    #     if "power_w" in telemetry:
    #         return float(telemetry["power_w"])
    #     voltage = float((status or {}).get("battery_v", (status or {}).get("battery_voltage", 0.0)))
    #     return voltage * self.current


class Servo(Effector):
    """A hobby servo. It turns to an angle you ask for and holds it there.

    Every MARLIN servo this season is positional, so `angle()` is the command
    you want.

    `angle()` does not jump the servo straight to the new angle. It records
    the target, and the TX loop walks the commanded position towards it at
    `set_speed()` degrees per second, so a big move is a smooth sweep rather
    than a slam.
    """

    # All servos are positional this season. Only angle() and micros() will be
    # available in the API. speed() is commented out to prevent confusion.
    #
    # Continuous-rotation calibration. Real servos idle off 1500us and have a
    # deadband; set per instance (`s.neutral_us = 1492`) after trimming.
    # neutral_us: int = 1500
    # span_us: int = 500
    #
    # def speed(self, pct: float) -> None:
    #     """Spin a continuous-rotation servo: -100 to 100, 0 to stop.
    #
    #     100 is full speed one way, -100 full speed the other. It keeps spinning
    #     until you tell it otherwise. Outside -100..100 raises ValueError.
    #
    #     For a servo that holds an angle instead, use `angle()`.
    #     """
    #     value = float(pct)
    #     if not -100.0 <= value <= 100.0:
    #         raise ValueError(f"{self.port}: servo speed must be between -100 and 100, got {pct}.")
    #     self.micros(int(round(self.neutral_us + value / 100.0 * self.span_us)))

    def __init__(self, port: str, state_mirror=None) -> None:
        super().__init__(port, state_mirror)
        self._speed: float = SERVO_SPEED_DEFAULT
        self._target_angle: float | None = None  # commanded angle; None = no sweep
        # Where the commanded position starts. There is no feedback, so this is
        # only ever what we last asked for - and an idle port asks for centre,
        # which is where the brain holds a servo until a phase commands one.
        self._current_pos: float = SERVO_ANGLE_CENTRE

    def set_speed(self, speed: float) -> None:

        value = float(speed)
        if not self._commandable(value, "speed"):
            return
        value = _clamp(value, SERVO_SPEED_MIN, SERVO_SPEED_MAX)
        self._speed = value

    def angle(self, deg: float) -> None:
        """Turn a positional servo to an angle in degrees: 0 to 180.

        The servo sweeps there at the speed set by `set_speed()` and holds it
        until you ask for a different angle. It takes a moment to arrive, so
        setting an angle does not mean it has got there yet. An angle past
        either end is pinned to that end - ask for 200 and the servo goes
        to 180.
        """
        value = float(deg)
        if not self._commandable(value, "angle"):
            return
        value = _clamp(value, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        # Recording the target is the whole job: `_move` walks the commanded
        # position towards it and emits the word the TX loop sends.
        self._target_angle = value

    def _move(self, dt: float) -> int | None:
        # Read the target once: angle() writes it from the student's thread
        # without this lock, and a single assignment is atomic, so we see either
        # the old value or the new one rather than something in between.
        target = self._target_angle
        if target is None:
            return None

        # Speed is deg/s, so `speed * dt` is the furthest this frame may move.
        # Clamping the remaining error to that budget saturates on a long move
        # and passes through untouched on the last one, landing exactly on
        # target with no overshoot.
        current = self._current_pos
        current += _clamp(target - current, -self._speed * dt, self._speed * dt)

        self._current_pos = current
        return encode_port_word(Tag.SERVO_POSITION, int(round(current * 10)))

    def _reset(self) -> None:
        # Drop the target so a sweep cannot outlive its phase, and park the
        # commanded position where an idle port actually leaves the servo.
        # Keeping the old position instead would have the next angle() sweep
        # from a value the hardware left behind the moment the phase ended, so
        # the servo would jump back to it before starting the new move.
        self._target_angle = None
        self._current_pos = SERVO_ANGLE_CENTRE

    # def micros(self, us: int) -> None:
    #     """Command the servo directly, in microseconds: 500 to 2500.

    #     Underneath, a servo is told what to do by the length of a repeating
    #     electrical pulse - roughly 1000 microseconds at one end of its travel,
    #     1500 in the middle and 2000 at the other end.

    #     `angle()` works this out for you. Reach for `micros()` only when
    #     calibrating a servo, or when you need something `angle()` will not give
    #     you. A pulse past either end is pinned to that end.
    #     """
    #     value = float(us)
    #     if not self._commandable(value, "micros"):
    #         return
    #     value = _clamp(value, SERVO_MICROS_MIN, SERVO_MICROS_MAX)
    #     self._buffer(Tag.SERVO_MICROS, int(round(value)))


# TODO(v1-deferred): I2t thermal protection. Setpoint clamp bounds peak current,
# not duration. Sustained near-clamp current into a stalled thruster can cook a
# motor; the 15A fuse never trips on it. No instrumentation in v1.
