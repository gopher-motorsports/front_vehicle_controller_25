"""
Export a trained model to a .fsaepol file for the FVC, and check the file
reproduces the trained policy.

    python deploy/tools/export_policy.py models/skidpad_pretrained
    python deploy/tools/export_policy.py models/autocross_combo_v3_best --out deploy/build/ax_v3.fsaepol

Reads the model's .zip, _vecnormalize.pkl and _config.json. The parity check
drives staged runs with the trained policy and compares its actions with the
file's forward pass on every step; the largest difference is printed and must
be below 1e-4.
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

TOOLS = os.path.dirname(os.path.abspath(__file__))
DEPLOY = os.path.dirname(TOOLS)
REPO = os.path.dirname(DEPLOY)
sys.path[:0] = [REPO, TOOLS]

from env import make_env                                   # noqa: E402
from sensor import ConeSensor                              # noqa: E402
from train import config_path, vecnormalize_path           # noqa: E402
import fsaepol                                             # noqa: E402

ACTIVATIONS = {torch.nn.Tanh: fsaepol.ACT_TANH, torch.nn.ReLU: fsaepol.ACT_RELU}


def _layers(policy):
    """[(W, b, activation)] for the deterministic actor: MLP then action head."""
    out, pending = [], None
    for m in policy.mlp_extractor.policy_net:
        if isinstance(m, torch.nn.Linear):
            if pending is not None:
                out.append(pending + (fsaepol.ACT_NONE,))
            pending = (m.weight.detach().numpy(), m.bias.detach().numpy())
        elif type(m) in ACTIVATIONS:
            out.append(pending + (ACTIVATIONS[type(m)],))
            pending = None
        else:
            raise ValueError(f"unsupported layer {m}")
    if pending is not None:
        out.append(pending + (fsaepol.ACT_NONE,))
    a = policy.action_net
    out.append((a.weight.detach().numpy(), a.bias.detach().numpy(), fsaepol.ACT_NONE))
    return out


def _sensor_meta(sensor):
    if isinstance(sensor, ConeSensor):
        return dict(range_scale_m=sensor.max_range, bearing_scale_rad=sensor.fov / 2,
                    filter_range_m=sensor.max_range, filter_half_fov_rad=sensor.fov / 2)
    # LidarCameraSensor: what it can report is bounded by the camera view and range
    return dict(range_scale_m=sensor.norm_range, bearing_scale_rad=sensor.bearing_scale,
                filter_range_m=min(sensor.camera_max_range, sensor.roi_forward + sensor.mount_x),
                filter_half_fov_rad=min(sensor.camera_half_fov, sensor.half_fov))


def export(model_path, out_path, name=None, parity_runs=3):
    from stable_baselines3 import PPO
    import pickle
    with open(config_path(model_path)) as f:
        cfg = json.load(f)
    if cfg.get("action_mode", "absolute") != "absolute":
        print(f"NOTE: trained in {cfg['action_mode']} mode; the controller must apply the "
              "same rate limits and report the applied commands")
    env = make_env(cfg["event"], sensor=cfg.get("sensor", "simple"),
                   action_mode=cfg.get("action_mode", "absolute"),
                   launch_assist_speed=cfg.get("launch_assist_speed", 3.0))
    model = PPO.load(model_path, device="cpu")
    with open(vecnormalize_path(model_path), "rb") as f:
        vn = pickle.load(f)
    p, tr = env.vehicle.p, env.track
    layers = _layers(model.policy)
    meta = dict(
        event=env.rules.name, name=name or os.path.basename(model_path),
        flags=fsaepol.FLAG_SQUASH if model.policy.squash_output else 0,
        top_k=env.sensor.top_k, slot_size=env.sensor.slot_size,
        clip_obs=float(vn.clip_obs), norm_eps=float(vn.epsilon),
        max_steer_rad=float(p.max_steer_angle), max_motor_torque=float(p.max_motor_torque),
        max_brake_torque=float(p.max_brake_torque), front_brake_bias=float(p.front_brake_bias),
        launch_assist_speed=float(cfg.get("launch_assist_speed", 3.0)),
        crossing_turns=[float(t) for t in tr.crossing_turns],
        action_mode=cfg.get("action_mode", "absolute"),
        steer_rate=float(env.limits.steer_rate), pedal_rate=float(env.limits.pedal_rate),
        driven_wheels=len(p.driven_wheels()),
        **_sensor_meta(env.sensor))
    std = np.sqrt(vn.obs_rms.var + vn.epsilon)
    blob = fsaepol.write(out_path, meta, vn.obs_rms.mean, std, layers)
    h, mean, std_r, layers_r = fsaepol.read(blob)

    # Parity: the file's forward pass against the trained policy, on real runs.
    worst, steps = 0.0, 0
    norm = lambda o: np.clip((o - vn.obs_rms.mean) / np.sqrt(vn.obs_rms.var + vn.epsilon),
                             -vn.clip_obs, vn.clip_obs).astype(np.float32)
    for s in range(parity_runs):
        obs, _ = env.reset(seed=s, options={"curriculum": False})
        for _ in range(env.max_steps):
            a_ref = model.predict(norm(obs), deterministic=True)[0]
            a_file = fsaepol.forward(h, mean, std_r, layers_r, obs)
            worst = max(worst, float(np.max(np.abs(a_ref - a_file))))
            steps += 1
            obs, _, term, trunc, _ = env.step(a_ref)
            if term or trunc:
                break
    print(f"Exported {model_path} -> {out_path}")
    print(f"  event {h['event']}, {h['obs_dim']} inputs, {len(layers_r)} layers "
          f"({', '.join(str(W.shape[0]) for W, _, _ in layers_r)}), {len(blob)} bytes, "
          f"CRC OK")
    print(f"  drive: {h['driven_wheels']} driven wheels in the simulator "
          f"({h['max_motor_torque']:g} N m each at full throttle)")
    print(f"  actions: {h['action_mode']} mode"
          + ("" if h["action_mode"] == "absolute" else
             f", steer rate {h['steer_rate']:g}/s, pedal rate {h['pedal_rate']:g}/s"))
    print(f"  sensor: range scale {h['range_scale_m']:.1f} m, bearing scale "
          f"{np.degrees(h['bearing_scale_rad']):.0f} deg, reports within "
          f"{h['filter_range_m']:.1f} m and +-{np.degrees(h['filter_half_fov_rad']):.0f} deg")
    if steps:
        print(f"  parity over {steps} steps: max |action difference| {worst:.2e} "
              + ("OK" if worst < 1e-4 else "FAILED"))
    if worst >= 1e-4:
        raise SystemExit(1)
    return blob


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", help="Model path without .zip")
    ap.add_argument("--out", default=None, help="Default: deploy/build/<event>.fsaepol")
    ap.add_argument("--name", default=None)
    ap.add_argument("--parity-runs", type=int, default=3)
    args = ap.parse_args()
    model = args.model[:-4] if args.model.endswith(".zip") else args.model
    with open(config_path(model)) as f:
        event = json.load(f)["event"]
    event = {"accel": "acceleration"}.get(event, event)
    out = args.out or os.path.join(DEPLOY, "build", f"{event}.fsaepol")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    export(model, out, args.name, args.parity_runs)


if __name__ == "__main__":
    main()
