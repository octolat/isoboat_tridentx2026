"""Student-facing Robot handle; see the package docstring for transport architecture.

Owns the link, config, state mirror, controller, and the configured
sensor/effector objects. All hardware hangs off this object.

Lifecycle (see spec sec.3):
    CONFIGURING --configure()--> CONFIGURED --run()--> [runtime: arm, dispatch
    phases, disarm]

Match phases (AUTON/DRIVER/PAUSE/ESTOP/...) originate from the field/brain OR
from the extension (which can drive a local-dev "field" from the IDE UI). The
library only reports the current phase via `robot.phase`.
"""

from __future__ import annotations

from collections import Counter
from typing import Callable

from . import _codec, _runtime
from .controller import Controller
from .effectors import Effector, Servo, Thruster
from .enums import Phase
from ._codec import encode_port_word
from ._link import Link, discover_endpoint
from ._state import StateMirror
from .sensors import IMU, RadioBeacon, Sensor


SENSOR_PORTS = {f"S{i}" for i in range(1, 7)}
EFFECTOR_PORTS = {f"M{i}" for i in range(1, 9)}
# How many of each sensor type a single robot may declare. The brain's beacon
# channel is per-config, so a second RadioBeacon has nothing to mean.
SENSOR_LIMITS = {IMU: 2, RadioBeacon: 1}
SUPPORTED_SENSORS = tuple(SENSOR_LIMITS)
SUPPORTED_EFFECTORS = (Thruster, Servo)


class Robot:
    """Your robot. Make one at the top of your program, and only one.

    Two steps get you going: `configure()` says what hardware is plugged into
    which port, then `run()` hands control to the functions you wrote for each
    part of the match. Everything else - motors, sensors, the controller -
    hangs off this object.

        robot = Robot()
        robot.configure(sensors={}, effectors={"M1": Thruster})
        robot.run(driver=driver)

    You do not normally call `arm()` or `disarm()` yourself; `run()` does.
    """

    def __init__(self, endpoint: str | None = None) -> None:
        """Connect to the MARLIN extension. Normally written as just `Robot()`.

        Your program does not talk to the robot directly. The MARLIN extension
        inside your editor holds the USB connection to the C3 box and passes
        messages both ways, so your program only ever talks to the extension.

        Args:
            endpoint: where to find the extension, e.g. "tcp://127.0.0.1:5172".
                Leave this out - the extension tells the library where it is
                when it launches your program.
        """
        self._endpoint = endpoint or discover_endpoint()
        self._state = StateMirror()
        self._link = Link(self._endpoint, self._state)
        self._phase: Phase = Phase.CONFIGURING
        self._sensors: dict[str, Sensor] = {}
        self._effectors: dict[str, Effector] = {}
        self._runtime_active_phase: Phase | None = None
        self._runtime_callback_active = False
        self._last_tick = 0.0  # last `robot.running` read; paces student loops
        self.controller: Controller = Controller(state_mirror=self._state)

    # ------------------------------------------------------------------ setup

    def configure(
        self,
        *,
        sensors: dict[str, type[Sensor]],
        effectors: dict[str, type[Effector]],
    ) -> None:
        """Say what hardware is plugged into which port. Call this once, first.

        Ports are the labelled sockets on the robot: S1-S6 take sensors, M1-M8
        take motors and servos. List only the ones you are using, and write the
        hardware type on its own - `Thruster`, not `Thruster()`.

            robot.configure(
                sensors={"S2": IMU},
                effectors={"M1": Thruster, "M5": Servo},
            )

        MARLIN sends the list to the robot's brain and waits for the brain to
        confirm it matches. That way a wrong port or a robot wired differently
        than you expected is an error here, at the top of your program, instead
        of hardware that quietly does nothing later.

        Args:
            sensors: port -> sensor type, e.g. {"S2": IMU}.
            effectors: port -> motor or servo type, e.g. {"M1": Thruster}.

        Raises:
            ValueError or TypeError: a port name or hardware type is not valid.
            RuntimeError: the robot has already started, or the brain reports
                different hardware than you declared.
        """
        if self._phase != Phase.CONFIGURING:
            raise RuntimeError("Robot.configure() is only legal before the robot is armed.")

        self._validate_config(sensors, effectors)

        self._link.open()
        self._link.time_sync()
        self._link.send(_codec.encode_config(sensors, effectors))
        echo = self._link.wait_for_config_echo()

        expected = _normalized_config(sensors, effectors)
        actual = _normalized_echo(echo)
        if actual != expected:
            raise RuntimeError(f"Robot configuration mismatch. Expected {expected}, got {actual}.")

        self._sensors = {str(port): cls(str(port), self._state) for port, cls in sensors.items()}
        self._effectors = {
            str(port): cls(str(port), self._state) for port, cls in effectors.items()
        }
        idle = encode_port_word(_codec.Tag.IDLE, 0)
        for port, effector in self._effectors.items():
            self._state.buffer_setpoint(port, idle)
            if isinstance(effector, Servo):
                self._state.register_mover(port, effector)

        self._phase = Phase.CONFIGURED
        self._state.update_phase(Phase.CONFIGURED)

    # ----------------------------------------------------------- arm / disarm

    def arm(self) -> None:
        """Allow the motors to move. You don't normally call this - `run()` does.

        Only useful if you are running the match phases yourself instead of
        using `run()`. Raises if `configure()` has not succeeded, or if the
        brain refuses to arm (while e-stopped or overheated, for example).
        """
        if self._phase != Phase.CONFIGURED:
            raise RuntimeError("Robot.arm() requires a successful Robot.configure() first.")
        self._link.send(_codec.encode_arm(True))
        self._link.wait_for_ack("arm")

    def disarm(self) -> None:
        """Stop the motors being allowed to move. `run()` does this at the end.

        Safe to call at any time, including when the robot has already gone
        away - it never raises.
        """
        try:
            self._link.send(_codec.encode_arm(False))
            self._link.wait_for_ack("disarm", timeout=0.5, raise_on_timeout=False)
        except Exception:
            pass

    # ----------------------------------------------------------------- run()

    def run(
        self,
        *,
        initialize: Callable[["Robot"], None] | None = None,
        autonomous: Callable[["Robot"], None] | None = None,
        driver: Callable[["Robot"], None] | None = None,
    ) -> None:
        """Start the match and hand control to the functions you wrote.

        You write one function per part of the match and pass them in here.
        MARLIN calls the right one when the match reaches that part:

            robot.run(autonomous=autonomous, driver=driver)

        Each function is handed the robot, and is called ONCE, at the moment
        its phase starts - so it has to keep looping until the phase ends:

            def driver(robot):
                while robot.running:
                    ...

        `run()` does not return until the match is over. Parts you leave out
        simply do nothing, and MARLIN prints a line in the console saying so
        when that part of the match comes up. Whenever a phase ends, or your
        function returns
        early, MARLIN stops every motor for you; a function that returns early
        is not called again for that phase, and one that keeps looping past the
        end of its phase is stopped and its commands ignored.

        Args:
            initialize: runs once, before autonomous, as soon as the robot is
                connected and its hardware checked. Setup only - reading
                sensors, printing, working out numbers you need later. Motor
                commands given here are thrown away when it returns, so put
                anything that has to move hardware at the start of
                autonomous() or driver() instead.
            autonomous: runs when the autonomous phase starts (no driver input).
            driver: runs when the driver-controlled phase starts.
        """
        _runtime.run_match(self, initialize, autonomous, driver)

    # ------------------------------------------------------------------ state

    @property
    def running(self) -> bool:
        """True while the current part of the match is still going. Loop on it.

            while robot.running:
                ...

        It turns False the moment the phase your function was called for ends,
        so the loop stops on its own and your function can finish.

        Reading it also paces your loop: it waits just long enough that the
        loop repeats 50 times a second, the rate MARLIN sends commands to the
        robot at. You never need to add a sleep of your own.
        """
        return _runtime.is_running(self)

    @property
    def phase(self) -> Phase:
        """Which part of the match the robot is in right now, as a `Phase`.

        Compare it against the names on `Phase`, e.g.
        `if robot.phase == Phase.AUTON:`. The field decides the phase; your
        program can read it but not change it.
        """
        return self._state.get_phase()

    @property
    def link_ok(self) -> bool:
        """True while the wireless link from the C3 box to the robot is healthy.

        False means your commands are not currently reaching the robot.
        """
        status = self._state.get_status() or {}
        return bool(status.get("link_ok", False))

    @property
    def estopped(self) -> bool:
        """True while the robot is emergency-stopped, with all motors cut.

        Only the MARLIN panel in your editor and the field can start or clear
        an emergency stop. Your program can check it but never change it.
        """
        status = self._state.get_status() or {}
        return bool(status.get("estopped", False))

    @property
    def overtemp(self) -> bool:
        """True while the robot's brain has got too hot and shut its motors off.

        It stops itself at 50 C and will not restart until the board has cooled
        to 45 C, at which point the extension restarts it for you. Read-only,
        like `estopped`.
        """
        status = self._state.get_status() or {}
        return bool(status.get("overtemp", False))

    @property
    def battery_voltage(self) -> float:
        """The robot battery's voltage in volts, from its last status message.

        Reads 0.0 before the robot has reported anything. A falling voltage
        means the battery is running down.
        """
        status = self._state.get_status() or {}
        return float(status.get("battery_v", 0.0) or 0.0)

    # ----------------------------------------------------------- accessors

    def sensor(self, port: str) -> Sensor:
        """Return the sensor plugged into `port`, e.g. `robot.sensor("S2")`.

        The port must be one you named in `configure()`; anything else raises
        KeyError.
        """
        return self._sensors[str(port)]

    def effector(self, port: str) -> Effector:
        """Return the motor or servo on `port`, e.g. `robot.effector("M1")`.

        The port must be one you named in `configure()`; anything else raises
        KeyError. `thruster()` and `servo()` are usually clearer.
        """
        return self._effectors[str(port)]

    def thruster(self, port: str) -> Thruster:
        """Return the thruster on `port`, ready to command.

            robot.thruster("M1").set_duty(50)

        Like `effector()`, but complains straight away (TypeError) if that port
        holds something other than a Thruster, so a mixed-up port is easy to
        spot.
        """
        eff = self.effector(port)
        if not isinstance(eff, Thruster):
            raise TypeError(f"{port} is configured as {eff.type}, not a Thruster.")
        return eff

    def servo(self, port: str) -> Servo:
        """Return the servo on `port`, ready to command.

            robot.servo("M5").angle(90)

        Like `effector()`, but raises TypeError if that port holds something
        other than a Servo.
        """
        eff = self.effector(port)
        if not isinstance(eff, Servo):
            raise TypeError(f"{port} is configured as {eff.type}, not a Servo.")
        return eff

    def imu(self, port: str | None = None) -> IMU:
        """Return an IMU so you can read it, e.g. `robot.imu().read()`.

        Leave `port` out when you configured only one IMU. With two, say which
        one: `robot.imu("S2")`.
        """
        return self._typed_sensor(IMU, "imu", port)

    def radiobeacon(self, port: str | None = None) -> RadioBeacon:
        """Return the radio beacon receiver, e.g. `robot.radiobeacon().read()`.

        `port` is optional, since a robot can only have one of these.
        """
        return self._typed_sensor(RadioBeacon, "radiobeacon", port)

    def _typed_sensor(self, cls: type, accessor: str, port: str | None) -> Sensor:
        """Back the typed sensor accessors. Omitted `port` resolves iff unambiguous."""
        if port is not None:
            found = self.sensor(port)
            if not isinstance(found, cls):
                raise TypeError(f"{port} is configured as {found.type}, not a {cls.__name__}.")
            return found

        matches = sorted(
            (s for s in self._sensors.values() if isinstance(s, cls)), key=lambda s: s.port
        )
        if len(matches) == 1:
            return matches[0]
        if not matches:
            raise LookupError(f"No {cls.__name__} is configured.")
        ports = ", ".join(s.port for s in matches)
        raise LookupError(
            f"{len(matches)} {cls.__name__}s are configured ({ports}); "
            f"say which one, e.g. robot.{accessor}({matches[0].port!r})."
        )

    def _validate_config(
        self,
        sensors: dict[str, type[Sensor]],
        effectors: dict[str, type[Effector]],
    ) -> None:
        for port, cls in sensors.items():
            if port not in SENSOR_PORTS:
                raise ValueError(
                    f"Invalid sensor port {port!r}. Expected one of "
                    f"{', '.join(sorted(SENSOR_PORTS))}."
                )
            if not isinstance(cls, type) or not issubclass(cls, Sensor):
                raise TypeError(f"{port}: sensor declaration must be a Sensor subclass.")
            if cls not in SUPPORTED_SENSORS:
                known = ", ".join(sensor.__name__ for sensor in SUPPORTED_SENSORS)
                raise TypeError(f"{port}: unsupported sensor type {cls.__name__}. Known types: {known}.")

        counts = Counter(sensors.values())
        for cls, limit in SENSOR_LIMITS.items():
            if counts[cls] > limit:
                ports = ", ".join(sorted(p for p, c in sensors.items() if c is cls))
                raise ValueError(
                    f"At most {limit} {cls.__name__} may be configured; "
                    f"got {counts[cls]} ({ports})."
                )

        for port, cls in effectors.items():
            if port not in EFFECTOR_PORTS:
                raise ValueError(
                    f"Invalid effector port {port!r}. Expected one of "
                    f"{', '.join(sorted(EFFECTOR_PORTS))}."
                )
            if not isinstance(cls, type) or not issubclass(cls, Effector):
                raise TypeError(f"{port}: effector declaration must be an Effector subclass.")
            if cls not in SUPPORTED_EFFECTORS:
                known = ", ".join(effector.__name__ for effector in SUPPORTED_EFFECTORS)
                raise TypeError(f"{port}: unsupported effector type {cls.__name__}. Known types: {known}.")


def _normalized_config(
    sensors: dict[str, type[Sensor]],
    effectors: dict[str, type[Effector]],
) -> dict:
    return {
        "sensors": {str(port): cls.__name__.lower() for port, cls in sorted(sensors.items())},
        "effectors": {str(port): cls.__name__.lower() for port, cls in sorted(effectors.items())},
    }


def _normalized_echo(echo: dict) -> dict:
    return {
        "sensors": {str(port): str(kind).lower() for port, kind in echo.get("sensors", {}).items()},
        "effectors": {
            str(port): str(kind).lower() for port, kind in echo.get("effectors", {}).items()
        },
    }


# TODO(v1-deferred): IMU-as-arming-prerequisite. Omitted for v1; if added later,
# enforce in arm() / configure() (block arming if an IMU is declared but its
# samples are stale/missing).
# TODO(v1-deferred): Brain motion primitives (HEADING_HOLD/TURN_TO/etc.) are
# dropped for v1. Mixing is student-side; there is intentionally no
# `drive(surge, yaw)` here.
