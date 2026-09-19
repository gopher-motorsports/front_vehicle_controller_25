"""
The .fsaepol model file: one trained policy, ready to flash to the vehicle
controller.

Everything the controller needs to reproduce the simulator's policy is inside:
the event, the observation normalisation, the sensor settings the policy was
trained with (so the controller filters and scales cones the same way), the
action scaling (what -1..1 means in steering angle and torque), the launch
assist speed, the event's timing-line sequence, and the network weights.

Layout (little-endian, every field 4-byte aligned so a controller can use the
file in place from memory-mapped flash):

    header      176 bytes, see HEADER_FMT and fsae_policy.h
    obs_mean    float32[obs_dim]
    obs_std     float32[obs_dim]      sqrt(var + eps), precomputed
    layers      n_layers times: uint32 in, uint32 out, uint32 activation,
                float32[out*in] weights (row-major), float32[out] bias
    crc32       uint32 over every byte before it (zlib.crc32)

driven_wheels: how many wheels max_motor_torque applies to in the simulator
(2 for rear-wheel drive), so total drive torque = per-wheel torque x this.
Action modes: 0 absolute, 1 target (the applied command moves toward the
action at steer_rate / pedal_rate full-scale units per second), 2 delta.
Activation codes: 0 none, 1 tanh, 2 relu. Header flag bit 0: apply tanh to the
output (squashed policies).
"""
import struct
import time
import zlib

import numpy as np

MAGIC = b"FSAEPOL1"
FORMAT_VERSION = 1
HEADER_FMT = "<8s9I11fI8f32sII2fI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 176
EVENT_IDS = {"skidpad": 0, "acceleration": 1, "autocross": 2}
EVENT_NAMES = {v: k for k, v in EVENT_IDS.items()}
ACT_NONE, ACT_TANH, ACT_RELU = 0, 1, 2
MAX_CROSSINGS = 8
FLAG_SQUASH = 1

HEADER_FIELDS = (
    "magic", "format_version", "total_size", "event", "obs_dim", "act_dim",
    "n_layers", "flags", "top_k", "slot_size",
    "range_scale_m", "bearing_scale_rad", "filter_range_m", "filter_half_fov_rad",
    "clip_obs", "norm_eps", "max_steer_rad", "max_motor_torque", "max_brake_torque",
    "front_brake_bias", "launch_assist_speed",
    "n_crossings", "crossing_turns", "name", "created_unix",
    "action_mode", "steer_rate", "pedal_rate", "driven_wheels",
)
ACTION_MODES = {"absolute": 0, "target": 1, "delta": 2}


def write(path, meta, obs_mean, obs_std, layers):
    """meta: dict of header values; layers: [(W, b, activation)]."""
    turns = list(meta["crossing_turns"])[:MAX_CROSSINGS]
    turns += [0.0] * (MAX_CROSSINGS - len(turns))
    body = bytearray()
    body += np.asarray(obs_mean, "<f4").tobytes()
    body += np.asarray(obs_std, "<f4").tobytes()
    for W, b, act in layers:
        W = np.asarray(W, "<f4")
        b = np.asarray(b, "<f4")
        body += struct.pack("<3I", W.shape[1], W.shape[0], act)
        body += W.tobytes() + b.tobytes()
    total = HEADER_SIZE + len(body) + 4
    header = struct.pack(
        HEADER_FMT, MAGIC, FORMAT_VERSION, total, EVENT_IDS[meta["event"]],
        len(obs_mean), layers[-1][0].shape[0], len(layers), meta.get("flags", 0),
        meta["top_k"], meta["slot_size"],
        meta["range_scale_m"], meta["bearing_scale_rad"], meta["filter_range_m"],
        meta["filter_half_fov_rad"], meta["clip_obs"], meta["norm_eps"],
        meta["max_steer_rad"], meta["max_motor_torque"], meta["max_brake_torque"],
        meta["front_brake_bias"], meta["launch_assist_speed"],
        len(meta["crossing_turns"]), *turns,
        meta["name"].encode()[:31].ljust(32, b"\0"), int(meta.get("created_unix", time.time())),
        ACTION_MODES[meta.get("action_mode", "absolute")],
        meta.get("steer_rate", 0.0), meta.get("pedal_rate", 0.0), meta.get("driven_wheels", 2))
    blob = header + bytes(body)
    blob += struct.pack("<I", zlib.crc32(blob) & 0xFFFFFFFF)
    assert len(blob) == total and total % 4 == 0
    with open(path, "wb") as f:
        f.write(blob)
    return blob


def read(path_or_bytes):
    data = path_or_bytes if isinstance(path_or_bytes, (bytes, bytearray)) else open(path_or_bytes, "rb").read()
    if data[:8] != MAGIC:
        raise ValueError("not an FSAE policy file")
    crc = struct.unpack_from("<I", data, len(data) - 4)[0]
    if zlib.crc32(data[:-4]) & 0xFFFFFFFF != crc:
        raise ValueError("CRC mismatch: file is corrupt")
    vals = struct.unpack_from(HEADER_FMT, data, 0)
    h = {}
    i = 0
    for name in HEADER_FIELDS:
        if name == "crossing_turns":
            h[name] = list(vals[i:i + MAX_CROSSINGS]); i += MAX_CROSSINGS
        else:
            h[name] = vals[i]; i += 1
    h["crossing_turns"] = h["crossing_turns"][: h["n_crossings"]]
    h["name"] = h["name"].rstrip(b"\0").decode()
    h["event"] = EVENT_NAMES[h["event"]]
    h["action_mode"] = {v: k for k, v in ACTION_MODES.items()}[h["action_mode"]]
    off = HEADER_SIZE
    n = h["obs_dim"]
    mean = np.frombuffer(data, "<f4", n, off); off += 4 * n
    std = np.frombuffer(data, "<f4", n, off); off += 4 * n
    layers = []
    for _ in range(h["n_layers"]):
        fin, fout, act = struct.unpack_from("<3I", data, off); off += 12
        W = np.frombuffer(data, "<f4", fin * fout, off).reshape(fout, fin); off += 4 * fin * fout
        b = np.frombuffer(data, "<f4", fout, off); off += 4 * fout
        layers.append((W, b, act))
    return h, mean, std, layers


def forward(h, mean, std, layers, obs):
    """Reference forward pass, the same arithmetic as fsae_policy.c."""
    x = np.clip((np.asarray(obs, np.float32) - mean) / std, -h["clip_obs"], h["clip_obs"])
    for W, b, act in layers:
        x = W @ x + b
        if act == ACT_TANH:
            x = np.tanh(x)
        elif act == ACT_RELU:
            x = np.maximum(x, 0)
    if h["flags"] & FLAG_SQUASH:
        x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)
