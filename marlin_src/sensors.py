"""Sensor base class, Reading base, and concrete sensor types.

Every read is non-blocking and returns the latest cached sample. Each reading
carries (timestamp, age, ok) so the student can decide whether a sample is fresh
enough to act on.

This iteration ships two sensor types: IMU and RadioBeacon.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


FRESH_SAMPLE_MS = 250.0


@dataclass
class Reading:
    """What every sensor reading carries, whichever sensor it came from.

    Attributes:
        timestamp: when the robot measured this, in milliseconds on the
            robot's own clock. Useful for comparing two readings, not for
            telling the time.
        age: how long ago that was, in milliseconds. It keeps growing until a
            newer reading arrives, so a big age means nothing recent has come
            through.
        ok: True if this reading is worth using - it exists and is less than
            250 ms old. Check it before trusting the numbers.
    """

    timestamp: int
    age: float
    ok: bool


@dataclass
class ImuReading(Reading):
    """One IMU reading: nine numbers describing how the robot is moving.

    Attributes:
        accel_x, accel_y, accel_z: acceleration along each axis, in g (1 g is
            the pull of gravity). Sitting still, the axis pointing down reads
            about 1 and the other two about 0.
        gyro_x, gyro_y, gyro_z: how fast the robot is turning about each axis,
            in degrees per second. All 0 when it is not turning.
        mag_x, mag_y, mag_z: the magnetic field along each axis, as whole
            numbers straight off the chip - no units, and not calibrated.

    It also carries timestamp, age and ok from `Reading`.
    """

    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: int
    mag_y: int
    mag_z: int


@dataclass
class RadioBeaconReading(Reading):
    """One reading from the radio beacon receiver.

    Attributes:
        rssi: how strong the received signal is, in dBm. The numbers are
            negative and closer to zero is stronger, so -40 is a far better
            signal than -100.
        snr: how far the signal stands out above background noise, in dB.
            Higher is better; negative means it is buried in noise.
        beacon_id: which beacon sent the message.
        rx_count: how many beacon messages the robot has received in total.

    It also carries timestamp, age and ok from `Reading`.
    """

    rssi: float
    snr: float
    beacon_id: int
    rx_count: int


class Sensor:
    """What every sensor has in common.

    You never make one of these yourself. You name the type in `configure()`
    (`sensors={"S2": IMU}`) and MARLIN builds the object for you; you get it
    back with `robot.sensor("S2")` or a shortcut like `robot.imu()`.

    Attributes:
        port: the port it is plugged into, e.g. "S2".
        type: its type name in lower case, e.g. "imu".
    """

    def __init__(self, port: str, state_mirror=None) -> None:
        """Bind this sensor to a port and the shared state mirror."""
        self.port: str = port
        self.type: str = self.__class__.__name__.lower()
        self._state = state_mirror

    def read(self) -> Reading:
        """Return the most recent reading. Comes back straight away.

        The robot sends readings continuously; this hands you the newest one
        that has arrived rather than waiting for a fresh one, so it is safe to
        call every time round your loop. If nothing has arrived yet, you get a
        reading with `ok` set to False.
        """
        sample = self._state.get_sensor(self.port) if self._state is not None else None
        age = self._state.sensor_age_ms(self.port) if self._state is not None else None
        if sample is None:
            return Reading(timestamp=0, age=math.inf, ok=False)
        timestamp, _payload = sample
        age_value = math.inf if age is None else float(age)
        return Reading(timestamp=timestamp, age=age_value, ok=age_value <= FRESH_SAMPLE_MS)


class IMU(Sensor):
    """Motion sensor: measures acceleration, turning and magnetic field.

    Name it in `configure()` (`sensors={"S2": IMU}`), then read it:

        reading = robot.imu().read()
        if reading.ok:
            print(reading.gyro_z)
    """

    def read(self) -> ImuReading:
        """Return the newest `ImuReading`. Comes back straight away.

        Holds acceleration, turn rate and magnetic field, plus timestamp, age
        and ok. Check `.ok` before using the numbers.
        """
        sample = self._state.get_sensor(self.port) if self._state is not None else None
        age = self._state.sensor_age_ms(self.port) if self._state is not None else None
        if sample is None:
            return ImuReading(
                timestamp=0,
                age=math.inf,
                ok=False,
                accel_x=0.0,
                accel_y=0.0,
                accel_z=0.0,
                gyro_x=0.0,
                gyro_y=0.0,
                gyro_z=0.0,
                mag_x=0,
                mag_y=0,
                mag_z=0,
            )

        timestamp, payload = sample
        age_value = math.inf if age is None else float(age)
        return ImuReading(
            timestamp=timestamp,
            age=age_value,
            ok=bool(payload.get("ok", True)) and age_value <= FRESH_SAMPLE_MS,
            accel_x=_axis(payload, "accel", "x", float),
            accel_y=_axis(payload, "accel", "y", float),
            accel_z=_axis(payload, "accel", "z", float),
            gyro_x=_axis(payload, "gyro", "x", float),
            gyro_y=_axis(payload, "gyro", "y", float),
            gyro_z=_axis(payload, "gyro", "z", float),
            mag_x=_axis(payload, "mag", "x", int),
            mag_y=_axis(payload, "mag", "y", int),
            mag_z=_axis(payload, "mag", "z", int),
        )


class RadioBeacon(Sensor):
    """Long-range radio receiver: listens for beacons and reports the signal.

    It tells you how strong a beacon's signal is and which beacon sent it -
    not a direction or a distance, though signal strength does tend to rise as
    you get closer.

    Name it in `configure()` (`sensors={"S1": RadioBeacon}`), then read it:

        reading = robot.radiobeacon().read()
        if reading.ok:
            print(reading.rssi)
    """

    def read(self) -> RadioBeaconReading:
        """Return the newest `RadioBeaconReading`. Comes back straight away.

        Holds rssi, snr, beacon_id and rx_count, plus timestamp, age and ok.
        Check `.ok` before using the numbers.
        """
        sample = self._state.get_sensor(self.port) if self._state is not None else None
        age = self._state.sensor_age_ms(self.port) if self._state is not None else None
        if sample is None:
            return RadioBeaconReading(
                timestamp=0,
                age=math.inf,
                ok=False,
                rssi=0.0,
                snr=0.0,
                beacon_id=0,
                rx_count=0,
            )

        timestamp, payload = sample
        age_value = math.inf if age is None else float(age)
        return RadioBeaconReading(
            timestamp=timestamp,
            age=age_value,
            ok=bool(payload.get("ok", True)) and age_value <= FRESH_SAMPLE_MS,
            rssi=float(payload.get("rssi", 0)),
            snr=float(payload.get("snr", 0)),
            beacon_id=int(payload.get("beacon_id", 0)),
            rx_count=int(payload.get("rx_count", 0)),
        )


def _axis(payload: dict, group: str, axis: str, cast):
    values = payload.get(group, [])
    index = {"x": 0, "y": 1, "z": 2}[axis]
    return cast(values[index]) if len(values) > index else cast(0)


