"""Internal IPC link; see the package docstring for transport architecture.

Responsibilities of this module:
- connect to the extension's local IPC endpoint (discovered via env var)
- background RX thread: parse incoming frames, dispatch by packet ID into the
  state mirror
- TX loop: emit the command frame at a fixed rate (keepalive on idle)
- counter sequencing for outgoing packets
- setup time-sync to capture laptop -> brain-ms offset (used by `Reading.age`)
  - the extension may forward sync packets transparently, or perform the sync
    itself and publish the offset to us; either way the library just consumes
    the resulting offset
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from urllib.parse import urlparse

from . import _codec
from ._codec import CRC, FRAME_HEADER, MAGIC, PacketID


# Environment variables the extension sets when it launches a student script.
# These let the library find the extension's IPC endpoint without hardcoding.
ENV_ENDPOINT = "MARLIN_LINK_ENDPOINT"        # e.g. "tcp://127.0.0.1:5172"
ENV_AUTH_TOKEN = "MARLIN_LINK_TOKEN"         # optional per-launch shared secret
DEFAULT_LOOPBACK_ENDPOINT = "tcp://127.0.0.1:5172"
CONNECT_TIMEOUT_S = 2.0
RESPONSE_TIMEOUT_S = 2.0
TX_RATE_HZ = 50.0


class LinkError(RuntimeError):
    """Raised when the MARLIN extension IPC link cannot complete an operation."""


def discover_endpoint() -> str:
    """Return the extension's IPC endpoint URL, or raise if not discoverable.

    Resolution order:
      1. $MARLIN_LINK_ENDPOINT (set by the extension on launch)
      2. a default loopback fallback (e.g. tcp://127.0.0.1:<default-port>) for
         out-of-IDE testing against a running extension instance
    """
    return os.environ.get(ENV_ENDPOINT) or DEFAULT_LOOPBACK_ENDPOINT


class Link:
    """IPC link to the extension + background RX/TX threads.

    Not student-facing. The Robot constructs one of these and hands a reference
    to the state mirror so packets are routed there as they arrive.

    The link does NOT touch the USB serial device - that is the extension's
    job. From this module's perspective, the extension is just "the other end
    of a socket that speaks the brain wire protocol".
    """

    def __init__(self, endpoint: str, state_mirror) -> None:
        """Store config; do NOT connect yet (open() does that).

        Args:
            endpoint: IPC URL of the extension (e.g. "tcp://127.0.0.1:5172").
            state_mirror: shared StateMirror; RX dispatches into it.
        """
        self._endpoint = endpoint
        self._state = state_mirror
        self._sock: socket.socket | None = None
        self._rx_thread: threading.Thread | None = None
        self._tx_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._tx_seq = 0
        self._brain_offset_ms: int | None = None
        self._auth_token = os.environ.get(ENV_AUTH_TOKEN)
        self._config_echo: dict | None = None
        self._config_event = threading.Event()
        self._ack_events: dict[str, threading.Event] = {}
        self._ack_results: dict[str, tuple[bool, str]] = {}
        self._opened = False

    def open(self) -> None:
        """Connect to the extension and start RX/TX background threads.

        Steps:
          1. parse self._endpoint -> (scheme, host, port)
          2. connect socket
          3. perform handshake (protocol version + optional auth token)
          4. start rx_thread and tx_thread
        """
        if self._opened:
            return

        host, port = self._parse_tcp_endpoint(self._endpoint)
        try:
            sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT_S)
        except OSError as exc:
            raise LinkError(
                f"Extension not reachable at {self._endpoint}. Is the MARLIN extension "
                "running, did it launch this script, and is the C3 plugged in?"
            ) from exc

        sock.settimeout(0.25)
        self._sock = sock
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="marlin-link-rx", daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, name="marlin-link-tx", daemon=True)
        self._rx_thread.start()

        hello = {
            "protocol": _codec.PROTOCOL_VERSION,
            "token": self._auth_token,
            "client": "marlin-python",
        }
        self.send(_codec.encode_frame(PacketID.HELLO, json.dumps(hello).encode("utf-8")))
        self.wait_for_ack("hello", timeout=RESPONSE_TIMEOUT_S)

        self._tx_thread.start()
        self._opened = True

    def close(self) -> None:
        """Stop background threads and close the socket."""
        self._stop.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        for thread in (self._rx_thread, self._tx_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=0.5)
        self._sock = None
        self._opened = False

    def send(self, payload: bytes) -> None:
        """Enqueue a single outgoing frame (config push, arm/disarm, etc.).

        The extension will relay this frame to the C3 over USB - or, for
        protocol packets the extension chooses to mediate (e.g. ARM_REQUEST
        while the IDE has a hold), it may handle / drop / delay the frame
        itself and synthesize the appropriate response.
        """
        sock = self._sock
        if sock is None:
            raise LinkError(
                f"Not connected to the MARLIN extension at {self._endpoint}. "
                "Call robot.configure() after starting the MARLIN extension."
            )

        frame = bytes(payload)
        if not frame.startswith(MAGIC):
            frame = _codec.encode_frame(PacketID.COMMAND, frame, self._tx_seq)

        try:
            with self._send_lock:
                sock.sendall(frame)
        except OSError as exc:
            raise LinkError(
                f"Lost connection to the MARLIN extension at {self._endpoint}. "
                "Check that the extension is still running and reconnect the C3 if needed."
            ) from exc

    def time_sync(self) -> int:
        """Run setup time-sync; cache laptop -> brain-ms offset; return it.

        The extension is in the path, so one of two things happens:
          (a) the extension forwards sync packets transparently and we sync
              with the brain end-to-end (extension adds ~constant latency), or
          (b) the extension performs the sync itself and pushes the resulting
              brain-time offset to us in-band.
        In either case the offset lands in the state mirror.
        """
        deadline = time.monotonic() + 0.25
        while self._brain_offset_ms is None and time.monotonic() < deadline:
            time.sleep(0.01)

        if self._brain_offset_ms is None:
            # Until the extension/brain pushes a precise offset, leave the
            # state mirror in receipt-time fallback mode. That gives students
            # useful non-negative Reading.age values in local simulator tests.
            return 0
        return self._brain_offset_ms

    @property
    def brain_offset_ms(self) -> int | None:
        """Current laptop -> brain-ms offset (set by time_sync); None before sync."""
        return self._brain_offset_ms

    def wait_for_config_echo(self, timeout: float = RESPONSE_TIMEOUT_S) -> dict:
        """Wait for the brain/extension to echo the parsed runtime config."""
        deadline = time.monotonic() + timeout
        config_ack_event = self._ack_events.setdefault("config", threading.Event())

        while not self._config_event.is_set():
            if config_ack_event.is_set():
                ok, message = self._ack_results.get("config", (True, ""))
                if not ok:
                    raise LinkError(message or "The robot rejected its configuration.")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._config_event.wait(min(0.05, remaining))

        if not self._config_event.is_set():
            raise LinkError(
                "Timed out waiting for the robot to confirm its configuration. "
                "Check that the MARLIN extension is connected to the C3 and the robot is powered."
            )
        if self._config_echo is None:
            raise LinkError("Received an empty configuration response from the robot.")
        return dict(self._config_echo)

    def wait_for_ack(
        self,
        request: str,
        timeout: float = RESPONSE_TIMEOUT_S,
        *,
        raise_on_timeout: bool = True,
    ) -> bool:
        """Wait for an ACK/NACK for a named request such as 'arm' or 'disarm'."""
        event = self._ack_events.setdefault(request, threading.Event())
        if not event.wait(timeout):
            if raise_on_timeout:
                raise LinkError(
                    f"Timed out waiting for {request} acknowledgement from the MARLIN extension. "
                    "Check the extension status and robot connection."
                )
            return False
        ok, message = self._ack_results.get(request, (True, ""))
        if not ok:
            raise LinkError(message or f"The robot rejected {request}.")
        return True

    # --- internals ----------------------------------------------------------

    def _rx_loop(self) -> None:
        """Background RX: read frames, decode, dispatch into state mirror by packet ID.

        Frames arriving here originate from either:
          - the robot brain (relayed by the extension over USB), OR
          - the extension itself (synthesized match-control packets: PHASE
            changes, E-Stop, time-offset push). The library does not need to
            distinguish - the packet IDs already tell it everything.
        """
        while not self._stop.is_set():
            try:
                frame = self._read_frame()
                if frame is None:
                    continue
                packet_id, body = _codec.decode_frame(frame)
            except socket.timeout:
                continue
            except (OSError, ValueError):
                # Socket failure or framing/CRC corruption: the byte stream can
                # no longer be trusted, so mark the link down and stop.
                if not self._stop.is_set():
                    status = self._state.get_status() or {}
                    status["link_ok"] = False
                    self._state.update_status(status)
                break

            # Per-frame payload problems must NOT kill the RX thread: the
            # extension/firmware may legitimately emit packet IDs this library
            # version does not know (spec: treat brain- and extension-sourced
            # packets identically; new control packets may appear). Skip and
            # keep receiving.
            try:
                self._dispatch_packet(PacketID(packet_id), body)
            except (ValueError, KeyError, TypeError):
                continue

    def _tx_loop(self) -> None:
        """Background TX: emit the current command frame at a fixed rate.

        Keepalives keep firing during PAUSE / between callbacks so the brain's
        signal-loss failsafe does not trip spuriously. The extension also
        watches keepalives from this side - if the student script crashes the
        extension stops forwarding setpoints and the brain failsafe takes over.
        """
        period = 1.0 / TX_RATE_HZ
        next_tick = time.monotonic()
        last_frame = time.monotonic()
        while not self._stop.is_set():
            # Real elapsed time, not the nominal period: a frame that overruns
            # would otherwise under-report how long it took, and a servo's
            # commanded position would slip behind the speed that was asked for.
            now = time.monotonic()
            self._state.tick(now - last_frame)
            last_frame = now

            frame = _codec.encode_command_frame(self._state.snapshot_setpoints(), self._tx_seq)
            self._tx_seq = (self._tx_seq + 1) & 0xFFFF
            try:
                self.send(frame)
            except LinkError:
                break

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_tick = time.monotonic()

    @staticmethod
    def _parse_tcp_endpoint(endpoint: str) -> tuple[str, int]:
        parsed = urlparse(endpoint)
        if parsed.scheme != "tcp" or not parsed.hostname or parsed.port is None:
            raise LinkError(
                f"Unsupported MARLIN endpoint {endpoint!r}. Expected a URL like "
                f"{DEFAULT_LOOPBACK_ENDPOINT}."
            )
        return parsed.hostname, parsed.port

    def _read_frame(self) -> bytes | None:
        header = self._recv_exact(FRAME_HEADER.size)
        if header is None:
            return None
        magic, _packet_id, _seq, body_len = FRAME_HEADER.unpack(header)
        if magic != MAGIC:
            raise ValueError("Invalid MARLIN frame magic")
        body_and_crc = self._recv_exact(body_len + CRC.size)
        if body_and_crc is None:
            return None
        return header + body_and_crc

    def _recv_exact(self, count: int) -> bytes | None:
        sock = self._sock
        if sock is None:
            return None
        chunks: list[bytes] = []
        remaining = count
        while remaining:
            if self._stop.is_set():
                return None
            chunk = sock.recv(remaining)
            if not chunk:
                raise OSError("socket closed")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _dispatch_packet(self, packet_id: PacketID, body: bytes) -> None:
        if packet_id == PacketID.STATUS:
            status = _codec.decode_status(body)
            self._state.update_status(status)
        elif packet_id == PacketID.SENSOR_BUNDLE:
            self._state.update_sensors(_codec.decode_sensor_bundle(body))
        elif packet_id == PacketID.CONTROLLER_INPUT:
            self._state.update_controller(_codec.decode_controller_input(body))
        elif packet_id == PacketID.CONFIG_ECHO:
            self._config_echo = _codec.decode_config_echo(body)
            self._config_event.set()
        elif packet_id == PacketID.PHASE:
            payload = json.loads(body.decode("utf-8")) if body else {}
            self._state.update_phase(payload.get("phase", payload))
        elif packet_id in (PacketID.ACK, PacketID.NACK):
            payload = json.loads(body.decode("utf-8")) if body else {}
            request = str(payload.get("request", "hello"))
            ok = packet_id == PacketID.ACK and bool(payload.get("ok", True))
            message = str(payload.get("message", ""))
            self._ack_results[request] = (ok, message)
            self._ack_events.setdefault(request, threading.Event()).set()
        elif packet_id == PacketID.TIME_OFFSET:
            payload = json.loads(body.decode("utf-8")) if body else {}
            offset = int(payload.get("offset_ms", 0))
            self._brain_offset_ms = offset
            self._state.set_brain_offset(offset)
