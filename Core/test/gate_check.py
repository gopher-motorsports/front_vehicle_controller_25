"""
Skidpad gate check: does a policy take the right way at each gate?

Spawns the car rolling late in each of the four laps (the same spawns training
uses) and drives until 3 s after it crosses the gate, or until the run ends.
The way the car went after the gate comes from its heading change since the
gate (more than 0.3 rad right or left, otherwise straight). Each attempt is
counted as:

    ok            right direction, still on course 3 s after the gate
    right, lost   right direction, but left the course, spun or stalled
    wrong dir     went a different way than the course
    before gate   the run ended before the gate
    completed     (last gate) came to a legal stop

"wrong dir" concentrated at one gate means that decision is not learned.
"right, lost" means the decision is fine and the car fails at driving the
circle. "before gate" means it failed earlier still.

Usage:
    python gate_check.py --model models/skidpad_v2_s1_best
    python gate_check.py --scripted          # reference driver, for comparison
"""
import argparse
import collections
import json
import os

import numpy as np

from env import make_env

def expected_move(turns, gate):
    """'continue right', 'switch to left', 'continue left', ... or 'exit'."""
    now, before = float(turns[gate]), float(turns[gate - 1])
    if now == 0.0:
        return "exit"
    side = "right" if now < 0 else "left"
    return f"continue {side}" if now == before else f"switch to {side}"


def load_policy(args, env):
    if args.scripted:
        from reference_driver import ReferenceDriver
        return ReferenceDriver(env, speed=9.5, ay_max=10.5)
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from train import vecnormalize_path
    model = PPO.load(args.model)
    stats_path = vecnormalize_path(args.model)
    norm = lambda o: o
    if os.path.exists(stats_path):
        stats = VecNormalize.load(stats_path, DummyVecEnv([lambda: env]))
        stats.training = False
        norm = stats.normalize_obs
    else:
        print(f"WARNING: no normalisation statistics at {stats_path}")
    return lambda o: model.predict(norm(o), deterministic=not args.stochastic)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None)
    ap.add_argument("--scripted", action="store_true")
    ap.add_argument("--attempts", type=int, default=20, help="per gate")
    ap.add_argument("--stochastic", action="store_true",
                    help="Sample actions instead of the deterministic policy")
    args = ap.parse_args()
    if not args.scripted and not args.model:
        ap.error("give --model or --scripted")

    kwargs = {}
    if args.model:
        from train import config_path
        cp = config_path(args.model)
        if os.path.exists(cp):
            with open(cp) as f:
                cfg = json.load(f)
            if cfg.get("event") != "skidpad":
                ap.error(f"{args.model} was trained on {cfg.get('event')}, not skidpad")
            kwargs = dict(sensor=cfg.get("sensor", "simple"),
                          action_mode=cfg.get("action_mode", "absolute"))
    env = make_env("skidpad", **kwargs)
    policy = load_policy(args, env)
    tr = env.track
    dt = env.control_dt

    cols = ("ok", "right, lost", "wrong dir", "before gate", "completed")
    print(f"{'gate':>4}  {'expected move':15s} " + " ".join(f"{c:>12s}" for c in cols))
    for k in range(len(tr.curriculum_ranges)):
        counts = collections.Counter()
        wrong_ways = collections.Counter()
        gate = k + 1                                  # crossing reached from loop k
        move = expected_move(tr.crossing_turns, gate)
        want = float(tr.crossing_turns[gate])         # -1 right, +1 left, 0 straight
        for a in range(args.attempts):
            obs, _ = env.reset(seed=1000 * (k + 1) + a,
                               options={"curriculum": True, "curriculum_range": k})
            yaw_at_gate = None
            status = None
            for step in range(env.max_steps):
                obs, _, term, trunc, info = env.step(policy(obs))
                yaw = float(env.vehicle.state[2])
                if yaw_at_gate is None and env._next_cross > gate:
                    yaw_at_gate, gate_step = yaw, step
                if term or trunc:
                    status = info["status"]
                    break
                if yaw_at_gate is not None and (step - gate_step) * dt >= 3.0:
                    status = "running"
                    break
            if status == "completed":
                counts["completed"] += 1
                continue
            if yaw_at_gate is None:
                counts["before gate"] += 1
                continue
            turned = yaw - yaw_at_gate
            went = -1.0 if turned < -0.3 else (1.0 if turned > 0.3 else 0.0)
            if went != want:
                counts["wrong dir"] += 1
                wrong_ways[{-1.0: "right", 1.0: "left", 0.0: "straight"}[went]] += 1
            elif status == "running":
                counts["ok"] += 1
            else:
                counts["right, lost"] += 1
        line = f"{gate:>4}  {move:15s} " + " ".join(f"{counts[c]:>12d}" for c in cols)
        if wrong_ways:
            line += "   wrong dir went: " + ", ".join(f"{w} {n}" for w, n in wrong_ways.items())
        print(line)


if __name__ == "__main__":
    main()
