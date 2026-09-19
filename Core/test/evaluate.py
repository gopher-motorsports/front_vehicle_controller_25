"""
Run one staged run of an event and save a diagnostic panel: the path against
the track and cones, speed against cross-track error, steering against yaw rate,
throttle and brake, and a summary with the scored result (timed segments, cones,
off courses, run time, penalties, corrected time).

Usage:
    python evaluate.py --event skidpad --out random_rollout.png         # random policy
    python evaluate.py --event autocross --model models/autocross_ppo   # trained model
    python evaluate.py --event accel --scripted --out reference.png     # reference driver
    python evaluate.py --event autocross --scripted --track-seed 7      # a specific course
    python evaluate.py --event skidpad --model m --grip-scale 0.89      # calibrated tire
    python evaluate.py --event autocross --model m --sensor lidar_camera

`rollout()` and `plot_panel()` are importable if you want to drive a different
policy, e.g. a scripted controller.
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from env import make_env, STATUS_COMPLETED
from tire import make_tire
from track_base import COLOR_BLUE, COLOR_YELLOW, COLOR_ORANGE, COLOR_ORANGE_LARGE

OUTCOMES = {
    STATUS_COMPLETED: "completed the run",
    "dnf_off_course": "off course (DNF)",
    "dnf_lost": "left the course (DNF)",
    "dnf_spin": "spun out (DNF)",
    "dnf_stall": "stalled (DNF)",
    "unsafe_stop": "unsafe stop (DQ)",
    "timeout": "ran out of time",
}

CONE_STYLE = {
    COLOR_BLUE: dict(c="tab:blue", s=18, label="blue (left)"),
    COLOR_YELLOW: dict(c="gold", s=18, label="yellow (right)", edgecolors="0.4", linewidths=0.3),
    COLOR_ORANGE: dict(c="tab:orange", s=14, label="small orange (lanes)"),
    COLOR_ORANGE_LARGE: dict(c="orangered", s=30, marker="^", label="large orange (timing)"),
}


def rollout(env, policy, seed=None, options=None):
    """Run one episode from staging. Returns per-step traces plus the result.

    `policy` takes an observation and returns an action.
    """
    options = dict(options or {})
    options.setdefault("curriculum", False)      # always a real, staged run
    obs, _ = env.reset(seed=seed, options=options)
    speeds, steers, ctes, yaw_rates, throttles, brakes = [], [], [], [], [], []
    total_reward = 0.0
    info = {}

    for _ in range(env.max_steps):
        action = policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

        _, _, _, vx, vy, r, *_ = env.vehicle.state
        speeds.append(float(np.hypot(vx, vy)))
        # Plot the commands actually applied (in delta mode the action is a rate).
        applied = getattr(env, "applied_action", action)
        steers.append(float(np.clip(applied[0], -1, 1)))
        # accel_norm is one channel: positive is throttle, negative is brake.
        accel = float(np.clip(applied[1], -1, 1))
        throttles.append(max(accel, 0.0))
        brakes.append(max(-accel, 0.0))
        ctes.append(float(info["cte"]))
        yaw_rates.append(float(r))
        if terminated or truncated:
            break

    status = info.get("status", "unknown")
    return {
        "speeds": np.array(speeds), "steers": np.array(steers), "ctes": np.array(ctes),
        "throttles": np.array(throttles), "brakes": np.array(brakes),
        "yaw_rates": np.array(yaw_rates), "total_reward": total_reward,
        "status": status, "outcome": OUTCOMES.get(status, status),
        "segment_times": info.get("segment_times", {}),
        "cones_hit": info.get("cones_hit", 0), "off_courses": info.get("off_courses", 0),
        "run_time": info.get("run_time"), "penalty_s": info.get("penalty_s", 0.0),
        "corrected_time": info.get("corrected_time"),
        "cone_hit_mask": env._cone_hit.copy(),
        "traj": np.array(env.trajectory),
    }


def _fmt(t):
    return "   --   " if t is None else f"{t:7.3f} s"


def plot_panel(env, res, title, out_path):
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1, 1])
    ax_track = fig.add_subplot(gs[0:2, 0])
    ax_speed = fig.add_subplot(gs[0:2, 1])
    ax_steer = fig.add_subplot(gs[2, 0])
    ax_pedal = fig.add_subplot(gs[3, 0], sharex=ax_steer)
    ax_summary = fig.add_subplot(gs[2:4, 1])
    fig.suptitle(title, fontsize=14)
    track, traj = env.track, res["traj"]

    # --- 1. track layout and path ---
    ax = ax_track
    ax.plot(track.cx, track.cy, "k--", lw=0.8, alpha=0.4, label="centerline")
    for color, style in CONE_STYLE.items():
        m = track.cone_color == color
        if m.any():
            ax.scatter(track.cone_xy[m, 0], track.cone_xy[m, 1], **style)
    hit = res.get("cone_hit_mask")
    if hit is not None and hit.any():
        ax.scatter(track.cone_xy[hit, 0], track.cone_xy[hit, 1], marker="x",
                   c="tab:red", s=45, label="cones hit", zorder=4)
    ax.plot(traj[:, 0], traj[:, 1], "-", c="tab:green", lw=1.8, label="vehicle path")
    ax.scatter([traj[0, 0]], [traj[0, 1]], c="tab:green", s=60, edgecolors="k",
               label="start", zorder=5)
    ax.scatter([traj[-1, 0]], [traj[-1, 1]], c="tab:red", s=60, edgecolors="k",
               label="end", zorder=5)
    if track.rules.name == "acceleration":
        ax.set_ylim(-8, 8)
    else:
        ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{track.rules.name} layout and vehicle path")
    ax.legend(loc="best", fontsize=7)
    ax.grid(True, alpha=0.3)

    dt = env.control_dt
    t = np.arange(len(res["speeds"])) * dt

    # --- 2. speed and cross-track error ---
    ax = ax_speed
    ax.plot(t, res["speeds"], c="tab:blue")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, res["ctes"], c="tab:orange", lw=1, alpha=0.8)
    for lim, style in ((env.legal_cte, "--"), (track.track_width / 2.0, ":")):
        ax2.axhline(lim, c="tab:red", ls=style, lw=1)
        ax2.axhline(-lim, c="tab:red", ls=style, lw=1)
    ax2.set_ylabel("cross-track error (m)", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title("Speed and CTE (dashed = legal line, dotted = lane edge)")

    # --- 3a. steer and yaw rate ---
    ax = ax_steer
    ax.plot(t, res["steers"], c="tab:blue", lw=1)
    ax.axhline(0.0, c="0.6", lw=0.8)
    ax.set_ylabel("steer", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax.tick_params(axis="x", labelbottom=False)
    ax.set_ylim(-1.15, 1.15)
    ax.grid(True, alpha=0.3)
    ax3 = ax.twinx()
    ax3.plot(t, res["yaw_rates"], c="tab:purple", lw=1, alpha=0.6)
    ax3.set_ylabel("yaw rate (rad/s)", color="tab:purple")
    ax3.tick_params(axis="y", labelcolor="tab:purple")
    ax.set_title("Steer and yaw rate")

    # --- 3b. throttle and brake ---
    ax = ax_pedal
    ax.fill_between(t, 0.0, res["throttles"], color="tab:green", alpha=0.7,
                    linewidth=0, label="throttle")
    ax.fill_between(t, 0.0, -res["brakes"], color="tab:red", alpha=0.7,
                    linewidth=0, label="brake")
    ax.axhline(0.0, c="0.6", lw=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pedal")
    ax.set_ylim(-1.15, 1.15)
    ax.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])
    ax.set_yticklabels(["1.0", "0.5", "0", "0.5", "1.0"])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    ax.set_title("Throttle and brake")

    # --- 4. summary ---
    ax = ax_summary
    ax.axis("off")
    rules = track.rules
    cte_abs = np.abs(res["ctes"])
    lines = [f"Event:              {rules.name}"]
    if rules.name == "autocross":
        lines.append(f"Course length:      {track.lap_length:.0f} m")
    lines += [f"Outcome:            {res['outcome']}", ""]
    for label, *_ in track.timed_segments:
        lines.append(f"{(label + ' (timed):'):20s}{_fmt(res['segment_times'].get(label))}")
    oc_rule = ("DNF" if rules.off_course_penalty_s is None
               else f"{rules.off_course_penalty_s:g} s each")
    lines += [
        f"Cones hit:          {res['cones_hit']}  ({rules.cone_penalty_s:g} s each)",
        f"Off course:         {res['off_courses']}  ({oc_rule})",
        f"Run time:           {_fmt(res['run_time'])}",
        f"Penalties:          {res['penalty_s']:7.3f} s",
        f"Corrected time:     {_fmt(res['corrected_time'])}",
        "",
        f"Episode length:     {len(res['speeds'])} steps ({len(res['speeds']) * dt:.2f} s)",
        f"Total reward:       {res['total_reward']:.1f}",
        f"Max speed:          {res['speeds'].max():.2f} m/s",
        f"Mean speed:         {res['speeds'].mean():.2f} m/s",
        f"Mean |CTE|:         {cte_abs.mean():.3f} m",
        f"Max |CTE|:          {cte_abs.max():.3f} m  (legal line {env.legal_cte:.2f} m)",
        f"Peak |yaw rate|:    {np.abs(res['yaw_rates']).max():.2f} rad/s",
        f"Throttle:           {100 * (res['throttles'] > 0).mean():.0f}% of steps, "
        f"mean {res['throttles'].mean():.2f}",
        f"Brake:              {100 * (res['brakes'] > 0).mean():.0f}% of steps, "
        f"mean {res['brakes'].mean():.2f}",
    ]
    ax.text(0.0, 0.98, "\n".join(lines), fontsize=10, va="top", family="monospace")
    ax.set_title("Summary")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", type=str, default=None,
                    choices=["skidpad", "accel", "acceleration", "autocross"],
                    help="Default: the model's training event, else skidpad")
    ap.add_argument("--model", type=str, default=None)
    ap.add_argument("--out", type=str, default="rollout.png")
    ap.add_argument("--scripted", action="store_true",
                    help="Use the ground-truth reference driver instead of a policy")
    ap.add_argument("--speed", type=float, default=30.0,
                    help="Speed cap for --scripted (m/s)")
    ap.add_argument("--ay-max", type=float, default=8.5,
                    help="Cornering acceleration for --scripted (m/s^2)")
    ap.add_argument("--track-seed", type=int, default=None,
                    help="Autocross course to run (default: drawn from --seed)")
    ap.add_argument("--sensor", type=str, default=None,
                    choices=["simple", "lidar_camera"],
                    help="Cone perception model (see sensor_lidar_camera.py)")
    ap.add_argument("--tire-npz", type=str, default=None)
    ap.add_argument("--grip-scale", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--action-mode", default=None, choices=["target", "delta", "absolute"],
                    help="Default: the model's training mode, else absolute")
    args = ap.parse_args()

    # Settings saved at training time fill in anything not given, and any
    # explicit setting that disagrees with them is flagged.
    cfg = {}
    if args.model:
        import json, os
        from train import config_path
        path = config_path(args.model)
        if os.path.exists(path):
            with open(path) as f:
                cfg = json.load(f)
            print(f"Training settings from {path}: event={cfg['event']}, "
                  f"sensor={cfg['sensor']}, action_mode={cfg['action_mode']}")
        else:
            print(f"WARNING: no training settings at {path}; check --event, "
                  "--sensor and --action-mode match how the model was trained")
    for key, fallback in (("event", "skidpad"), ("sensor", "simple"),
                          ("action_mode", "absolute")):
        given = getattr(args, key)
        trained = cfg.get(key)
        if given is None:
            setattr(args, key, trained or fallback)
        elif trained is not None and given != trained and not (
                {given, trained} <= {"accel", "acceleration"}):
            print(f"WARNING: --{key.replace('_', '-')} {given} but the model was "
                  f"trained with {trained}")

    env = make_env(args.event, sensor=args.sensor, action_mode=args.action_mode,
                   launch_assist_speed=cfg.get("launch_assist_speed", 3.0),
                   tire=make_tire(args.tire_npz, args.grip_scale))

    if args.scripted:
        from reference_driver import ReferenceDriver
        policy = ReferenceDriver(env, speed=args.speed, ay_max=args.ay_max)
        label = "(reference driver)"
    elif args.model:
        import os
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        from train import vecnormalize_path
        model = PPO.load(args.model)
        stats_path = vecnormalize_path(args.model)
        if os.path.exists(stats_path):
            # Only the observation scaling is needed; the stats file carries it.
            stats = VecNormalize.load(stats_path, DummyVecEnv([lambda: env]))
            stats.training = False
            policy = lambda o: model.predict(stats.normalize_obs(o), deterministic=True)[0]
            print(f"Using normalisation statistics from {stats_path}")
        elif cfg.get("normalize", False):
            raise SystemExit(f"ERROR: the model was trained with normalisation but "
                             f"{stats_path} is missing. It will not drive without it.")
        else:
            print(f"WARNING: no normalisation statistics at {stats_path}; "
                  "using raw observations")
            policy = lambda o: model.predict(o, deterministic=True)[0]
        label = f"({args.model})"
    else:
        policy = lambda o: env.action_space.sample()
        label = "(random policy)"

    options = {}
    if args.track_seed is not None:
        options["track_seed"] = args.track_seed
    res = rollout(env, policy, seed=args.seed, options=options)
    print(f"{env.rules.name}: episode ended after {len(res['speeds'])} steps -- {res['outcome']}")
    for label_, t in res["segment_times"].items():
        print(f"  {label_}: {t:.3f} s")
    print(f"  cones {res['cones_hit']}, off course {res['off_courses']}, "
          f"run {_fmt(res['run_time'])}, penalties {res['penalty_s']:.3f} s, "
          f"corrected {_fmt(res['corrected_time'])}")

    plot_panel(env, res, f"{env.rules.name.capitalize()} rollout {label}", "results/" + args.out)
    print(f"Saved plot to {args.out}")


if __name__ == "__main__":
    main()
