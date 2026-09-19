"""
Encode the perception message for the FVC (see firmware/fsae_msg.h).
Use this on the onboard computer after cone detection:

    from perception_msg import encode
    payload = encode(cones, crossings=n_lines, age_ms=latency_ms, vy=None)
    uart.write(frame(payload))          # over a UART: add the 0xA5 0x5A + length framing
    # cones: iterable of (x_m, y_m, cls), vehicle frame at capture, origin at the CG
    # cls: 0 blue, 1 yellow, 2 small orange, 3 large orange
"""
import struct

VERSION = 1
MAX_CONES = 64


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(cones, crossings, age_ms, vy=None):
    cones = sorted(cones, key=lambda c: c[0] ** 2 + c[1] ** 2)[:MAX_CONES]
    flags = 1 if vy is not None else 0
    vy_cm = int(round((vy or 0.0) * 100))
    out = struct.pack("<BBBBHh", VERSION, len(cones), min(int(crossings), 255), flags,
                      min(max(int(age_ms), 0), 65535), max(-32768, min(32767, vy_cm)))
    for x, y, cls in cones:
        out += struct.pack("<hhBB", max(-32768, min(32767, int(round(x * 100)))),
                           max(-32768, min(32767, int(round(y * 100)))), int(cls), 0)
    return out + struct.pack("<H", crc16(out))


def frame(payload):
    """Frame a message for a byte stream (UART): 0xA5 0x5A, u16 length, payload."""
    return b"\xa5\x5a" + struct.pack("<H", len(payload)) + payload
