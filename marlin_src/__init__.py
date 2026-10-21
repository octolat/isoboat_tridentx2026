"""MARLIN student library - groundstation-side Python API.

Public surface (everything a student needs):

    from marlin import Robot
    from marlin import IMU, RadioBeacon               # sensor type tokens
    from marlin import Thruster, Servo                # effector type tokens
    from marlin import Phase                          # for `robot.phase` comparisons

Transport architecture:
    The library does NOT speak USB. The MARLIN extension (VSCode / Claude Code
    plugin) owns the C3 serial port and relays the wire protocol over a local
    IPC socket; this library is its IPC client. That lets the extension also
    mediate match control (start/stop/E-Stop) without the student script in
    the loop. See `_link` for the IPC details.

Internals live in modules prefixed with an underscore (`_link`, `_codec`,
`_state`, `_runtime`). They are not part of the public API.
"""

from .controller import Button, Controller, Joystick
from .effectors import Effector, Servo, Thruster
from .enums import Phase, Tag
from .robot import Robot
from .sensors import IMU, ImuReading, RadioBeacon, RadioBeaconReading, Reading, Sensor

__all__ = [
    # top-level handle
    "Robot",
    # controller
    "Controller",
    "Joystick",
    "Button",
    # sensors (type tokens used in configure())
    "Sensor",
    "Reading",
    "IMU",
    "ImuReading",
    "RadioBeacon",
    "RadioBeaconReading",
    # effectors (type tokens used in configure())
    "Effector",
    "Thruster",
    "Servo",
    # enums
    "Phase",
    "Tag",
]
