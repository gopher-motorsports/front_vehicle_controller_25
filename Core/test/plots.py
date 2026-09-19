"""
Plots for fvc_sil.py: the evaluate.py panel, controller diagnostics, and exact
vs realistic comparison. Each function takes the trace dict fvc_sil.py records.
"""
import os

import numpy as np

def _panel_result(tr):
    """The result dict evaluate.plot_panel expects."""
    from evaluate import OUTCOMES
    info = tr["info"]
    applied = tr["applied"]
    return {
        "speeds": np.hypot(tr["vx"], tr["vy"]), "steers": applied[:, 0], "ctes": tr["cte"],
        "throttles": np.clip(applied[:, 1], 0, None), "brakes": np.clip(-applied[:, 1], 0, None),
        "yaw_rates": tr["r"], "total_reward": tr["total_reward"], "status": info["status"],
        "outcome": OUTCOMES.get(info["status"], info["status"]),
        "segment_times": info.get("segment_times", {}), "cones_hit": info.get("cones_hit", 0),
        "off_courses": info.get("off_courses", 0), "run_time": info.get("run_time"),
        "penalty_s": info.get("penalty_s", 0.0), "corrected_time": info.get("corrected_time"),
        "cone_hit_mask": tr["cone_hit_mask"], "traj": tr["traj"],
    }


def _setting_text(tr, settings):
    if tr["mode"] == "exact":
        return "true INS velocities, fresh cone list every step"
    source = "INS" if settings["estimator"] == "ins" else "wheel-speed + IMU"
    return (f"{source} velocities, cone list every "
            f"{20 * settings['frame_every']} ms, {settings['latency_ms']} ms latency")


def plot_run(env, tr, model_path, plot_dir, settings):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import plot_panel
    os.makedirs(plot_dir, exist_ok=True)
    name = os.path.basename(model_path)
    stem = os.path.join(plot_dir, f"{name}_{tr['mode']}_seed{tr['seed']}")
    title = f"FVC SIL {tr['mode']}: {name} ({_setting_text(tr, settings)})"
    plot_panel(env, _panel_result(tr), title, stem + "_run.png")

    t = tr["t"]
    fig, axs = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    fig.suptitle(f"FVC controller diagnostics: {name}, {tr['mode']}, seed {tr['seed']}\n"
                 f"{_setting_text(tr, settings)}", fontsize=12)

    ax = axs[0]
    ax.plot(t, tr["vx"], c="tab:blue", lw=1.5, label="vx true")
    ax.plot(t, tr["vx_est"], c="tab:blue", ls="--", lw=1, label="vx used by controller")
    ax.set_ylabel("vx (m/s)")
    ax2 = ax.twinx()
    ax2.plot(t, tr["vy"], c="tab:orange", lw=1, label="vy true")
    ax2.plot(t, tr["vy_est"], c="tab:orange", ls="--", lw=1, label="vy used")
    ax2.set_ylabel("vy (m/s)", color="tab:orange")
    err = np.abs(tr["vx_est"] - tr["vx"])
    ax.set_title(f"Ego state (max |vx error| {np.nanmax(err):.2f} m/s, "
                 f"max |vy| {np.nanmax(np.abs(tr['vy'])):.2f} m/s)")
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="lower right", ncol=4)
    ax.grid(alpha=0.3)

    ax = axs[1]
    age = 1000 * tr["frame_age"]
    ax.plot(t, age, c="tab:purple", lw=0.5, alpha=0.8, label="age of the cone list in use")
    arrivals = tr["arrivals"]
    if 0 < len(arrivals) <= 60:
        ax.plot(arrivals, np.zeros(len(arrivals)), "|", c="tab:green", ms=8, label="frame arrivals")
    stale = tr["stale"].astype(bool)
    if stale.any():
        ax.fill_between(t, 0, 1, where=stale, transform=ax.get_xaxis_transform(),
                        color="tab:red", alpha=0.15, label="perception stale (brake)")
    n_arr = len(arrivals) if len(arrivals) else len(t)
    ax.set_ylabel("frame age (ms)")
    ax.set_ylim(0, max(np.nanmax(age) * 1.3, 10) if np.isfinite(np.nanmax(age)) else 10)
    ax.set_title(f"Perception timing: {n_arr} frames, age {np.nanmin(age):.0f}-"
                 f"{np.nanmax(age):.0f} ms (mean {np.nanmean(age):.0f} ms)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3)

    ax = axs[2]
    ax.step(t, tr["cones_used"], where="post", c="tab:gray", lw=1)
    ax.set_ylabel("cones in observation")
    assist = tr["assist"].astype(bool)
    if assist.any():
        ax.fill_between(t, 0, 1, where=assist, transform=ax.get_xaxis_transform(),
                        color="tab:green", alpha=0.2, label="launch assist")
        ax.legend(fontsize=8, loc="lower right")
    ax.set_ylim(-0.5, 8.5)
    ax.set_title("Cones given to the policy")
    ax.grid(alpha=0.3)

    ax = axs[3]
    act, app = tr["action"], tr["applied"]
    ax.plot(t, act[:, 0], c="tab:blue", lw=0.8, alpha=0.5, label="steer: policy")
    ax.plot(t, app[:, 0], c="tab:blue", lw=1.2, label="steer: applied")
    ax.plot(t, act[:, 1], c="tab:red", lw=0.8, alpha=0.5, label="pedal: policy")
    ax.plot(t, app[:, 1], c="tab:red", lw=1.2, label="pedal: applied")
    title = "Commands (policy output and what the controller applied)"
    if tr["mode"] == "exact":
        diff = np.abs(act - tr["ref_action"]).max(axis=1)      # NaN where not compared
        axd = ax.twinx()
        axd.semilogy(t, np.maximum(diff, 1e-9), c="k", lw=0.6, alpha=0.6)
        axd.set_ylabel("|C - Python| action", fontsize=8)
        if np.isfinite(diff).any():
            title += f"; max difference from Python {np.nanmax(diff):.1e}"
    ax.set_ylim(-1.1, 1.1)
    ax.set_ylabel("command")
    ax.set_xlabel("time (s)")
    ax.set_title(title)
    ax.legend(fontsize=8, loc="lower left", ncol=4)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(stem + "_controller.png", dpi=110)
    plt.close(fig)


def plot_comparison(pairs, model_path, plot_dir):
    """Exact and realistic runs of the same seed, on one track plot."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import CONE_STYLE
    name = os.path.basename(model_path)
    for seed, (ex, re_) in pairs.items():
        track = ex["track"]
        fig, (ax, axs) = plt.subplots(1, 2, figsize=(15, 6.5),
                                      gridspec_kw=dict(width_ratios=[1, 1.2]))
        ax.plot(track.cx, track.cy, "k--", lw=0.6, alpha=0.4)
        for color, style in CONE_STYLE.items():
            m = track.cone_color == color
            if m.any():
                ax.scatter(track.cone_xy[m, 0], track.cone_xy[m, 1], **{**style, "s": 10})
        for tr, c, lab in ((ex, "tab:green", "exact"), (re_, "tab:purple", "realistic")):
            info = tr["info"]
            ct = info.get("corrected_time")
            ax.plot(tr["traj"][:, 0], tr["traj"][:, 1], c=c, lw=1.5,
                    label=f"{lab}: {info['status']}"
                          + (f", {ct:.2f} s" if ct is not None else "")
                          + f", {info.get('cones_hit', 0)} cones")
            hit = tr["cone_hit_mask"]
            if hit.any():
                ax.scatter(track.cone_xy[hit, 0], track.cone_xy[hit, 1], marker="x", c=c, s=40)
            axs.plot(tr["t"], np.hypot(tr["vx"], tr["vy"]), c=c, lw=1.2, label=f"{lab} speed")
        if track.rules.name == "acceleration":
            ax.set_ylim(-8, 8)
        else:
            ax.set_aspect("equal")
        ax.legend(fontsize=8, loc="best")
        ax.set_title("Path (x = cones hit)")
        ax.grid(alpha=0.3)
        axs.set_xlabel("time (s)")
        axs.set_ylabel("speed (m/s)")
        axs.set_title("Speed")
        axs.legend(fontsize=8)
        axs.grid(alpha=0.3)
        fig.suptitle(f"FVC SIL exact vs realistic: {name}, seed {seed}")
        fig.tight_layout()
        path = os.path.join(plot_dir, f"{name}_compare_seed{seed}.png")
        fig.savefig(path, dpi=110)
        plt.close(fig)


