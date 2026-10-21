"""Enums for MARLIN library: match phases and effector command tags."""

from enum import Enum, IntEnum


class Phase(Enum):
    """Which part of the match the robot is in, as decided by the field.

    Read it with `robot.phase`; your program can never set it.

        CONFIGURING - your program has started but has not configured yet.
        CONFIGURED  - the hardware is declared and the robot agrees; waiting.
        AUTON       - autonomous: your code drives, with no driver input.
        PAUSE       - the match is paused between phases; motors are stopped.
        DRIVER      - driver-controlled: the control panel is live.
        ESTOP       - emergency stop; motors are cut until it is cleared.
        POSTMATCH   - the match is over.
    """

    CONFIGURING = "configuring"
    CONFIGURED = "configured"
    AUTON = "auton"
    PAUSE = "pause"
    DRIVER = "driver"
    ESTOP = "estop"
    POSTMATCH = "postmatch"


class Tag(IntEnum):
    """3-bit effector command tag (top 3 bits of the 16-bit per-port command word).

    Wire format per port (16-bit LE):
        [15:13] tag (3 bits)
        [12:0]  value (13-bit signed, two's complement)

    decode: tag=(w>>13)&7; raw=w&0x1FFF; val = raw-0x2000 if raw&0x1000 else raw
    """

    IDLE = 0
    THRUSTER_DUTY = 1        # value: -100..100 (duty %)
    RESERVED_2 = 2           # retired THRUSTER_CURRENT: PID left the brain, tag unused on the wire
    SERVO_POSITION = 3       # value: 0..1800 (x0.1 deg -> 0..180.0 deg)
    SERVO_MICROS = 4         # value: 500..2500 (us pulse)
