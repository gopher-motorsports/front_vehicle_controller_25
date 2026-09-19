"""
Compare saved models on the same footing: each is driven deterministically from
a staged start on several seeds (for autocross, several different courses), and
the results are summarised in one table.

A single evaluate.py plot is one run on one seed; this is the number to trust
when choosing between models.

Usage:
    python compare_models.py models/skidpad_bc models/skidpad_best
    python compare_models.py --event autocross --seeds 10          # every autocross model
    python compare_models.py --all --json results/compare.json

Models are loaded with their saved settings (event, sensor, action mode) and
normalisation statistics, as evaluate.py does. Models whose observation size
does not match the current code are listed as incompatible.
"""
import argparse
import glob
import json
import os

import numpy as np

from env import make_env
from evaluate import rollout
from train import config_path, vecnormalize_path

EVENT_ALIASES = {"accel": "acceleration"}


def _event(name):
    return EVENT_ALIASES.get(name, name)


def find_models(event=None):
    """Final, best and pretrained models under models/ (not checkpoints)."""
    out = []
    for cfg in sorted(glob.glob("models/*_config.json")):
        base = cfg[: -len("_config.json")]
        if "_ckpt" in os.path.basename(base) or not os.path.exists(base + ".zip"):
            continue
        with open(cfg) as f:
            ev = _event(json.load(f).get("event", ""))
        if event is None or ev == _event(event):
            out.append(base)
    return out


def load(model_path):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    with open(config_path(model_path)) as f:
        cfg = json.load(f)
    env = make_env(cfg["event"], sensor=cfg.get("sensor", "simple"),
                   action_mode=cfg.get("action_mode", "absolute"),
                   launch_assist_speed=cfg.get("launch_assist_speed", 3.0))
    model = PPO.load(model_path)
    if model.observation_space.shape != env.observation_space.shape:
        return cfg, env, None
    norm = lambda o: o
    stats = vecnormalize_path(model_path)
    if os.path.exists(stats):
        vn = VecNormalize.load(stats, DummyVecEnv([lambda: env]))
        vn.training = False
        norm = vn.normalize_obs
    policy = lambda o: model.predict(norm(o), deterministic=True)[0]
    return cfg, env, policy


def evaluate_model(model_path, seeds):
    cfg, env, policy = load(model_path)
    row = {"model": model_path, "event": _event(cfg["event"]),
           "action_mode": cfg.get("action_mode"), "sensor": cfg.get("sensor")}
    if policy is None:
        row["incompatible"] = True
        return row
    runs = []
    for s in seeds:
        res = rollout(env, policy, seed=s)
        tr = env.track
        span = tr.s[tr.finish_idx] - tr.s[tr.start_idx]
        runs.append({
            "seed": s, "status": res["status"], "corrected": res["corrected_time"],
            "run_time": res["run_time"], "cones": res["cones_hit"],
            "off_courses": res["off_courses"],
            "distance": float(np.clip((env._prev_s - tr.s[tr.start_idx]) / span, 0, 1)),
        })
    done = [r for r in runs if r["corrected"] is not None]
    row.update({
        "runs": runs, "completed": len(done), "n": len(runs),
        "corrected_mean": float(np.mean([r["corrected"] for r in done])) if done else None,
        "corrected_best": float(np.min([r["corrected"] for r in done])) if done else None,
        "cones_mean": float(np.mean([r["cones"] for r in done])) if done else None,
        "distance_mean": float(np.mean([r["distance"] for r in runs])),
        "failures": sorted({r["status"] for r in runs if r["corrected"] is None}),
    })
    return row


def print_table(rows):
    print(f"{'model':34s} {'mode':8s} {'done':>6s} {'mean':>8s} {'best':>8s} "
          f"{'cones':>6s} {'dist':>5s}  failures")
    for r in rows:
        if r.get("incompatible"):
            print(f"{r['model']:34s} {'':8s} incompatible observation size")
            continue
        f = lambda v, p=2: "--" if v is None else f"{v:.{p}f}"
        print(f"{r['model']:34s} {str(r['action_mode']):8s} {r['completed']:>2d}/{r['n']:<3d} "
              f"{f(r['corrected_mean']):>8s} {f(r['corrected_best']):>8s} "
              f"{f(r['cones_mean'], 1):>6s} {r['distance_mean']:>5.2f}  "
              f"{', '.join(r['failures'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="*", help="Model paths (without .zip)")
    ap.add_argument("--event", default=None, help="Every model for this event")
    ap.add_argument("--all", action="store_true", help="Every model under models/")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--json", default=None, help="Also write results here")
    args = ap.parse_args()
    paths = [m[:-4] if m.endswith(".zip") else m for m in args.models]
    if args.all or args.event:
        paths += find_models(None if args.all else args.event)
    if not paths:
        ap.error("give model paths, --event or --all")
    seeds = list(range(args.seeds))
    rows = [evaluate_model(p, seeds) for p in paths]
    rows.sort(key=lambda r: (r["event"], r["model"]))
    for ev in sorted({r["event"] for r in rows}):
        print(f"\n{ev}  ({args.seeds} staged runs each"
              + (", one course per seed)" if ev == "autocross" else ")"))
        print_table([r for r in rows if r["event"] == ev])
    if args.json:
        os.makedirs(os.path.dirname(args.json) or ".", exist_ok=True)
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=2)


if __name__ == "__main__":
    main()
