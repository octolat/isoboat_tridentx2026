"""Internal wire codec; see the package docstring for transport architecture.

Encoders for outgoing frames (command, config push, arm/disarm, time-sync).
Decoders for incoming frames (status, sensor bundle, controller input, config
echo, phase, ack/nack).

The 3-bit-tag effector field (see marlin.enums.Tag) is implemented here as a
BITFIELD - it must NOT be decoded as a plain int16. Masking the 3-bit tag and
sign-extending the 13-bit value is mandatory.
"""

from __future__ import annotations

import json
import struct
import zlib
from enum import IntEnum
from typing import Any

from .enums import Tag


MAGIC = b"ML"
PROTOCOL_VERSION = 1
FRAME_HEADER = struct.Struct("<2sBHH")
CRC = struct.Struct("<I")


class PacketID(IntEnum):
    """Packet IDs used on the extension IPC channel."""

    HELLO = 1
    COMMAND = 2
    CONFIG = 3
    ARM_REQUEST = 4
    STATUS = 5
    SENSOR_BUNDLE = 6
    CONTROLLER_INPUT = 7
    CONFIG_ECHO = 8
    PHASE = 9
    ACK = 10
    NACK = 11
    TIME_OFFSET = 12

# --- effector command field (per port, 16-bit LE) ----------------------------
#
#   [15:13] tag (3 bits)
#   [12:0]  value (13-bit signed, two's complement)
#
# encode: w = ((tag & 7) << 13) | (value & 0x1FFF)
# decode: tag = (w >> 13) & 7
#         raw = w & 0x1FFF
#         val = raw - 0x2000 if (raw & 0x1000) else raw
# -----------------------------------------------------------------------------


def encode_port_word(tag: Tag, value: int) -> int:
    """Pack a (tag, value) pair into the 16-bit per-port command word.

    Args:
        tag: one of the Tag enum members.
        value: signed integer in the tag's documented range. Caller is
            responsible for clamping to 13-bit signed (-4096..4095); this
            function masks to 13 bits and trusts the caller.

    Returns:
        16-bit unsigned integer suitable for little-endian packing.
    """
    return ((int(tag) & 0x7) << 13) | (int(value) & 0x1FFF)


def decode_port_word(word: int) -> tuple[Tag, int]:
    """Unpack a 16-bit per-port command word into (tag, signed_value).

    Implements the BITFIELD decode (3-bit tag + sign-extended 13-bit value), not
    a plain int16 cast.
    """
    word = int(word) & 0xFFFF
    tag = Tag((word >> 13) & 0x7)
    raw = word & 0x1FFF
    val = raw - 0x2000 if (raw & 0x1000) else raw
    return tag, val


def encode_frame(packet_id: PacketID | int, body: bytes = b"", seq: int = 0) -> bytes:
    """Wrap a packet body with MARLIN IPC framing and CRC.

    Frame layout:
        magic(2) packet_id(u8) seq(u16) body_len(u16) body crc32(u32)

    The body is intentionally opaque to the framing layer. The v1 Python IPC
    helpers below use compact JSON bodies for config/status-style packets while
    preserving the required 16-bit effector command words inside that JSON.
    """
    body = bytes(body)
    header = FRAME_HEADER.pack(MAGIC, int(packet_id), int(seq) & 0xFFFF, len(body))
    checksum = zlib.crc32(header[2:] + body) & 0xFFFFFFFF
    return header + body + CRC.pack(checksum)


def _json_body(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _decode_json_body(body: bytes) -> dict:
    if not body:
        return {}
    decoded = json.loads(body.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError("Expected JSON object body")
    return decoded


# --- frame encoders ----------------------------------------------------------


def encode_command_frame(port_words: dict[str, int], seq: int) -> bytes:
    """Encode the full per-tick command frame.

    Args:
        port_words: {port_name: 16-bit encoded word} from encode_port_word().
            Missing ports are treated as IDLE.
        seq: outgoing sequence counter.
    """
    body = _json_body(
        {
            "seq": int(seq) & 0xFFFF,
            "ports": {str(port): int(word) & 0xFFFF for port, word in port_words.items()},
        }
    )
    return encode_frame(PacketID.COMMAND, body, seq)


def encode_config(sensors: dict, effectors: dict) -> bytes:
    """Serialize a Robot.configure() declaration for push to the brain."""
    def type_name(cls_or_name: object) -> str:
        return cls_or_name if isinstance(cls_or_name, str) else cls_or_name.__name__.lower()

    body = _json_body(
        {
            "sensors": {str(port): type_name(cls) for port, cls in sorted(sensors.items())},
            "effectors": {str(port): type_name(cls) for port, cls in sorted(effectors.items())},
        }
    )
    return encode_frame(PacketID.CONFIG, body)


def encode_arm(arm: bool) -> bytes:
    """Encode an ARM (True) or DISARM (False) request."""
    return encode_frame(PacketID.ARM_REQUEST, _json_body({"arm": bool(arm)}))


# --- frame decoders ----------------------------------------------------------


def decode_frame(buf: bytes) -> tuple[int, bytes]:
    """Strip framing/CRC and return (packet_id, body). Raises on CRC failure."""
    if len(buf) < FRAME_HEADER.size + CRC.size:
        raise ValueError("Incomplete MARLIN frame")

    magic, packet_id, _seq, body_len = FRAME_HEADER.unpack(buf[: FRAME_HEADER.size])
    if magic != MAGIC:
        raise ValueError("Invalid MARLIN frame magic")

    expected_len = FRAME_HEADER.size + body_len + CRC.size
    if len(buf) != expected_len:
        raise ValueError(f"Invalid MARLIN frame length: got {len(buf)}, expected {expected_len}")

    body_start = FRAME_HEADER.size
    body_end = body_start + body_len
    body = buf[body_start:body_end]
    (expected_crc,) = CRC.unpack(buf[body_end:])
    actual_crc = zlib.crc32(buf[2:body_end]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("MARLIN frame CRC mismatch")

    return packet_id, body


def decode_status(body: bytes) -> dict:
    """Decode a STATUS body -> {battery_v, link_ok, estopped, phase, ts_ms, ...}."""
    return _decode_json_body(body)


def decode_sensor_bundle(body: bytes) -> dict:
    """Decode the per-tick sensor bundle -> {port: payload_struct}."""
    bundle = _decode_json_body(body)
    samples = bundle.get("samples", bundle)
    if not isinstance(samples, dict):
        raise ValueError("Sensor bundle must contain a samples object")
    return samples


def decode_controller_input(body: bytes) -> dict:
    """Decode a CONTROLLER_INPUT body -> stick axes (raw + scaled) and button mask."""
    ctrl = _decode_json_body(body)
    ctrl.setdefault("left_stick_x_raw", 2048)
    ctrl.setdefault("left_stick_y_raw", 2048)
    ctrl.setdefault("right_stick_x_raw", 2048)
    ctrl.setdefault("right_stick_y_raw", 2048)
    ctrl.setdefault("left_stick_x_scaled", 0.0)
    ctrl.setdefault("left_stick_y_scaled", 0.0)
    ctrl.setdefault("right_stick_x_scaled", 0.0)
    ctrl.setdefault("right_stick_y_scaled", 0.0)
    ctrl.setdefault("buttons", {})
    # Triggers ride their own byte, outside the button mask, so they are folded in
    # here under their panel names. The lower-case keys read here are the
    # controller-side wire spelling; the upper-case ones written are what a
    # student types into controller.button().
    ctrl["buttons"]["LEFT_TRIGGER"] = bool(ctrl.get("left_trigger", False))
    ctrl["buttons"]["RIGHT_TRIGGER"] = bool(ctrl.get("right_trigger", False))
    return ctrl


def decode_config_echo(body: bytes) -> dict:
    """Decode the brain's parsed-config echo for verification against the local declaration."""
    return _decode_json_body(body)
