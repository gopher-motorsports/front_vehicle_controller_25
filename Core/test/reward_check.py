"""
Check that the reward ranks behaviours the right way round, using scripted
drivers: faster complete runs above slower ones, clean runs above faster runs
that cut through cones, completing above not stopping, any crash above
stalling. Prints undiscounted and discounted returns.

Usage:
    python reward_check.py              # gamma 0.999, as train.py uses
    python reward_check.py --gamma 0.995
"""
import argparse
import numpy as np

from env import make_env
from reference_driver import ReferenceDriver


def episode(event, make_policy, seed=0, gamma=0.999, **env_kwargs):
    env = make_env(event, curriculum_prob=0.0, **env_kwargs)
    obs, _ = env.reset(seed=seed)
    policy = make_policy(env)
    rewards, info = [], {}
    for _ in range(env.max_steps):
        obs, r, term, trunc, info = env.step(policy(obs))
        rewards.append(r)
        if term or trunc:
            break
    rewards = np.array(rewards)
    disc = float(np.sum(rewards * gamma ** np.arange(len(rewards))))
    t = info.get("corrected_time") or info.get("run_time")
    return info["status"], t, info.get("cones_hit", 0), float(rewards.sum()), disc


def ref(**kw):
    return lambda env: ReferenceDriver(env, **kw)


def no_stop(**kw):
    def make(env):
        d = ReferenceDriver(env, **kw)
        def act(obs):
            a = d(obs)
            if env._next_cross >= env.n_crossings:
                a = d._emit(float(env.applied_action[0]), 1.0)   # keep the throttle down
            return a
        return act
    return make


def crash_after(crossings, **kw):
    def make(env):
        d = ReferenceDriver(env, **kw)
        def act(obs):
            a = d(obs)
            if env._next_cross >= crossings:
                a = d._emit(1.0, float(env.applied_action[1]))    # full left lock
            return a
        return act
    return make


def stall(env):
    return lambda obs: np.array([0.0, -1.0], dtype=np.float32)


SCENARIOS = {
    "skidpad": [
        ("reference, fast", ref(speed=9.5, ay_max=10.5)),
        ("reference, slow", ref(speed=7.0)),
        ("faster, 6 cones", ref(speed=10.0, ay_max=11.0, line_offset=0.7)),
        ("faster, 18 cones", ref(speed=10.0, ay_max=11.0, line_offset=0.8)),
        ("no stop after finish", no_stop(speed=9.5, ay_max=10.5)),
        ("crash after 2 laps", crash_after(3, speed=9.5, ay_max=10.5)),
        ("crash at the gate", crash_after(1, speed=8.0)),
        ("stall at the start", lambda env: stall(env)),
    ],
    "accel": [
        ("reference, full speed", ref()),
        ("reference, cap 14 m/s", ref(speed=14.0)),
        ("reference, cap 10 m/s", ref(speed=10.0)),
        ("no stop after finish", no_stop()),
        ("crash at 30 m", lambda env: crash_after(1)(env) if False else _veer(env, 30.0)),
        ("stall at the start", lambda env: stall(env)),
    ],
    "autocross": [
        ("reference, ay 8.5", ref(ay_max=8.5)),
        ("reference, ay 5", ref(ay_max=5.0)),
        ("cutting cones", ref(ay_max=8.5, line_offset=0.8, full_offset_radius=15)),
        ("no stop after finish", no_stop()),
        ("crash at 100 m", lambda env: _veer(env, 100.0)),
        ("crash at 20 m", lambda env: _veer(env, 20.0)),
        ("stall at the start", lambda env: stall(env)),
    ],
}


def _veer(env, at_m):
    d = ReferenceDriver(env)
    s0 = env.track.s[env._track_idx]
    def act(obs):
        a = d(obs)
        if env.track.s[env._track_idx] - s0 > at_m:
            a = d._emit(1.0, 1.0)
        return a
    return act


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gamma", type=float, default=0.999)
    ap.add_argument("--events", nargs="*", default=list(SCENARIOS))
    args = ap.parse_args()
    for event in args.events:
        print(f"\n{event}  (gamma {args.gamma})")
        print(f"  {'scenario':24s} {'status':15s} {'time':>7s} {'cones':>5s} {'return':>8s} {'discounted':>11s}")
        for name, make in SCENARIOS[event]:
            status, t, cones, ret, disc = episode(event, make, gamma=args.gamma)
            ts = f"{t:7.2f}" if t is not None else "     --"
            print(f"  {name:24s} {status:15s} {ts} {cones:5d} {ret:8.1f} {disc:11.1f}")


if __name__ == "__main__":
    main()
