"""Internal: latest-value state mirror.

Holds the most recent cached payloads for:
  - robot status (BLE-sourced via the C3 bridge)
  - per-port sensor samples (BLE-sourced)
  - controller input (USB-sourced, direct from C3)
  - pending effector setpoints (per port, encoded 16-bit words)
  - latest phase

All student reads hit this. Two independent "ages" are tracked: one for the
BLE-sourced path (status + sensors) and one for the USB-sourced controller
path - they can fail independently.
"""

from __future__ import annotations

import time
import threading

from ._codec import encode_port_word
from .enums import Phase, Tag


def _now_ms() -> float:
    return time.time() * 1000.0


class StateMirror:
    """Thread-safe latest-value cache.

    The RX thread writes; the student's thread (via Sensor.read / Controller
    properties / Robot properties) reads. The TX thread reads pending setpoints.
    """

    def __init__(self) -> None:
        """Initialise empty caches and the locks that protect them."""
        self._lock = threading.RLock()
        self._status: dict | None = None
        self._sensors: dict[str, tuple[int, dict]] = {}
        self._sensor_receipts_ms: dict[str, float] = {}
        self._controller: dict | None = None
        self._setpoints: dict[str, int] = {}
        self._movers: dict[str, object] = {}  # port -> effector with _move/_reset
        self._setpoint_owner: int | None = None
        self._disowned_warned: set[int] = set()
        self._phase: Phase = Phase.CONFIGURING
        self._brain_offset_ms: int | None = None
        self._ble_last_rx_ms: float | None = None
        self._usb_last_rx_ms: float | None = None

    # --- writes (called from RX thread) -------------------------------------

    def update_status(self, status: dict) -> None:
        """Store the latest STATUS payload (battery, link_ok, estopped, ...)."""
        snapshot = dict(status)
        with self._lock:
            self._status = snapshot
            self._ble_last_rx_ms = _now_ms()
            if "phase" in snapshot:
                self._phase = _coerce_phase(snapshot["phase"], self._phase)

    def update_sensors(self, samples: dict) -> None:
        """Store the latest per-port sensor samples."""
        receipt = _now_ms()
        with self._lock:
            for port, sample in samples.items():
                payload = dict(sample) if isinstance(sample, dict) else {"value": sample}
                timestamp = int(payload.get("timestamp", 0))
                self._sensors[str(port)] = (timestamp, payload)
                self._sensor_receipts_ms[str(port)] = receipt
            self._ble_last_rx_ms = receipt

    def update_controller(self, ctrl: dict) -> None:
        """Store the latest CONTROLLER_INPUT payload (USB-sourced, direct from C3)."""
        snapshot = dict(ctrl)
        # Button lookups uppercase the requested name (see Controller.button), so
        # the cached keys must be uppercase too. Producers disagree on case - the
        # firmware sends UP/A/B, the codec adds LEFT_TRIGGER/RIGHT_TRIGGER - and a
        # mixed-case key silently never matches, which is exactly what made the
        # triggers look dead. Normalise once here so every producer is covered.
        buttons = snapshot.get("buttons")
        if isinstance(buttons, dict):
            snapshot["buttons"] = {str(n).upper(): bool(down) for n, down in buttons.items()}
        with self._lock:
            self._controller = snapshot
            self._usb_last_rx_ms = _now_ms()

    def update_phase(self, phase) -> None:
        """Store the latest match phase."""
        with self._lock:
            self._phase = _coerce_phase(phase, self._phase)

    def set_brain_offset(self, offset_ms: int) -> None:
        """Cache the laptop -> brain-ms offset captured by Link.time_sync()."""
        with self._lock:
            self._brain_offset_ms = int(offset_ms)

    # --- writes (called from Effector.set_*) --------------------------------

    def claim_setpoints(self) -> None:
        """Make the calling thread the only accepted source of setpoint writes.

        The runtime claims on behalf of each phase callback, then re-claims when
        the phase ends - so a callback that blocks past its own phase can no
        longer move the robot, even though Python cannot kill its thread.
        """
        with self._lock:
            self._setpoint_owner = threading.get_ident()

    def buffer_setpoint(self, port: str, encoded_word: int) -> None:
        """Buffer the encoded 16-bit command word for a port; flushed by TX loop.

        Writes from any thread other than the current owner are dropped (see
        `claim_setpoints`). Before the first claim there is no owner and all
        writes are accepted, so `configure()` can seed IDLE.
        """
        ident = threading.get_ident()
        with self._lock:
            if self._setpoint_owner is not None and ident != self._setpoint_owner:
                # ponytail: warn once per offending thread, drop the write. Hits
                # students who drive effectors from their own threads too; the
                # fix there is the same - write from the phase callback.
                warn = ident not in self._disowned_warned
                self._disowned_warned.add(ident)
            else:
                warn = None
                self._setpoints[str(port)] = int(encoded_word) & 0xFFFF

        if warn is not None:
            if warn:
                print(f"[API] ignoring {port} setpoint from code that outlived its phase.")
            return

    def zero_setpoints(self) -> None:
        """Reset all effector setpoints to IDLE. Called by runtime on callback return."""
        idle = encode_port_word(Tag.IDLE, 0)
        with self._lock:
            for port in list(self._setpoints):
                self._setpoints[port] = idle
            for mover in self._movers.values():
                mover._reset()

    # --- motion (called from TX thread) --------------------------------------

    def register_mover(self, port: str, effector) -> None:
        """Let the TX loop drive `effector._move(dt)` each frame for `port`."""
        with self._lock:
            self._movers[str(port)] = effector

    def tick(self, dt: float) -> None:
        """Advance every registered mover by `dt` seconds and buffer its word.

        Writes go straight into `_setpoints`: this is the library moving the
        robot on the student's behalf, not a student thread, so the owner
        check in `buffer_setpoint` does not apply.
        """
        with self._lock:
            for port, mover in self._movers.items():
                word = mover._move(dt)
                if word is not None:
                    self._setpoints[port] = word

    # --- reads (called from student thread / TX thread) ---------------------

    def get_status(self) -> dict | None:
        """Return the latest cached status payload, or None if none yet."""
        with self._lock:
            return None if self._status is None else dict(self._status)

    def get_sensor(self, port: str) -> tuple[int, dict] | None:
        """Return (timestamp_ms, payload) for the port, or None."""
        with self._lock:
            sample = self._sensors.get(str(port))
            if sample is None:
                return None
            timestamp, payload = sample
            return timestamp, dict(payload)

    def get_controller(self) -> dict | None:
        """Return the latest cached controller payload."""
        with self._lock:
            return None if self._controller is None else dict(self._controller)

    def get_phase(self):
        """Return the latest cached match phase."""
        with self._lock:
            return self._phase

    def snapshot_setpoints(self) -> dict[str, int]:
        """Return a shallow copy of pending setpoints for the TX loop."""
        with self._lock:
            return dict(self._setpoints)

    # --- derived ------------------------------------------------------------

    def sensor_age_ms(self, port: str) -> float | None:
        """Compute age (ms) of the latest sample on `port` using the brain offset."""
        port = str(port)
        with self._lock:
            sample = self._sensors.get(port)
            if sample is None:
                return None

            timestamp, _payload = sample
            if self._brain_offset_ms is not None:
                return _now_ms() - self._brain_offset_ms - timestamp

            receipt = self._sensor_receipts_ms.get(port)
            if receipt is None:
                return None
            return _now_ms() - receipt


def _coerce_phase(value, fallback: Phase) -> Phase:
    if isinstance(value, Phase):
        return value
    try:
        return Phase(str(value).lower())
    except ValueError:
        return fallback
