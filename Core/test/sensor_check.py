"""
Characterise a cone sensor while the reference driver laps some courses:
detection probability against range, position error against range, false
positives, colour errors and data age. Saves a comparison plot.

Usage:
    python sensor_check.py --out sensor_check.png            # both sensors
    python sensor_check.py --event skidpad --courses 1
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from env import make_env
from reference_driver import ReferenceDriver
from sensor import ConeSensor

BINS = np.arange(0.0, 21.0, 1.0)


def _all_detected(sensor):
    """Indices of every object the sensor currently reports, before top_k."""
    if isinstance(sensor, ConeSensor):
        return np.flatnonzero(sensor._prev_hit)
    pub = sensor._published
    return np.zeros(0, dtype=int) if pub is None else pub["ids"]


def _decode(sensor, row):
    """(range m, bearing rad) of one output row."""
    if isinstance(sensor, ConeSensor):
        return row[0] * sensor.max_range, row[1] * sensor.fov / 2.0
    return row[0] * sensor.norm_range, row[1] * np.pi


def characterise(sensor_name, event="autocross", courses=3, cone_half_angle=45.0):
    seen = np.zeros(len(BINS) - 1)
    found = np.zeros(len(BINS) - 1)
    err_sum = np.zeros(len(BINS) - 1)
    err_n = np.zeros(len(BINS) - 1)
    frames = fps = slots = color_err = 0
    ages = []
    for course in range(courses):
        env = make_env(event, sensor=sensor_name, curriculum_prob=0.0)
        obs, _ = env.reset(seed=course)
        driver = ReferenceDriver(env)
        sensor, tr = env.sensor, env.track
        n_cones = len(tr.cone_xy)
        for _ in range(env.max_steps):
            obs, _, term, trunc, _ = env.step(driver(obs))
            x, y, yaw, *_ = env.vehicle.state
            dx, dy = tr.cone_xy[:, 0] - x, tr.cone_xy[:, 1] - y
            bx = dx * np.cos(yaw) + dy * np.sin(yaw)
            by = -dx * np.sin(yaw) + dy * np.cos(yaw)
            dist = np.hypot(bx, by)
            ahead = np.abs(np.degrees(np.arctan2(by, bx))) <= cone_half_angle
            det = np.zeros(n_cones, dtype=bool)
            ids = _all_detected(sensor)
            det[ids[ids < n_cones]] = True
            b = np.digitize(dist, BINS) - 1
            ok = ahead & (b >= 0) & (b < len(BINS) - 1)
            np.add.at(seen, b[ok], 1)
            np.add.at(found, b[ok & det], 1)

            frames += 1
            fps += int(np.sum(ids >= n_cones))
            age = getattr(sensor, "last_age", None)
            ages.append(0.0 if isinstance(sensor, ConeSensor) else (np.nan if age is None else age))
            for slot, i in enumerate(sensor.last_truth):
                if i >= n_cones:
                    continue
                row = env.last_detections[slot]
                r, br = _decode(sensor, row)
                px, py = r * np.cos(br), r * np.sin(br)
                e = np.hypot(px - bx[i], py - by[i])
                k = int(np.clip(np.digitize(dist[i], BINS) - 1, 0, len(BINS) - 2))
                err_sum[k] += e
                err_n[k] += 1
                slots += 1
                color_err += int(np.argmax(row[2:6]) != tr.cone_color[i])
            if term or trunc:
                break
    with np.errstate(invalid="ignore", divide="ignore"):
        return {
            "p_detect": found / seen, "pos_err": err_sum / err_n,
            "fp_per_frame": fps / max(frames, 1),
            "color_err": color_err / max(slots, 1),
            "age_mean": float(np.nanmean(ages)),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", default="autocross")
    ap.add_argument("--courses", type=int, default=3)
    ap.add_argument("--out", default="sensor_check.png")
    args = ap.parse_args()

    centers = 0.5 * (BINS[:-1] + BINS[1:])
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    for name, color in (("simple", "tab:gray"), ("lidar_camera", "tab:blue")):
        res = characterise(name, args.event, args.courses)
        print(f"{name:13s} false positives/frame {res['fp_per_frame']:.3f}  "
              f"colour errors {100 * res['color_err']:.1f}%  "
              f"mean data age {1000 * res['age_mean']:.0f} ms")
        for r0, p in zip(BINS[:-1], res["p_detect"]):
            pass
        axs[0].plot(centers, res["p_detect"], "o-", c=color, label=name)
        axs[1].plot(centers, res["pos_err"], "o-", c=color, label=name)
        print("  P(detect) by range: " + " ".join(
            f"{int(c)}m:{p:.2f}" for c, p in zip(BINS[:-1], res["p_detect"]) if np.isfinite(p)))
    axs[0].set(xlabel="range from CG (m)", ylabel="P(cone reported)",
               title="Detection probability, cones within 45 deg of heading", ylim=(0, 1.05))
    axs[1].set(xlabel="range from CG (m)", ylabel="mean position error (m)",
               title="Reported position error (includes data age)")
    for ax in axs:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.suptitle(f"Sensor check: reference driver, {args.event}, {args.courses} course(s)")
    fig.tight_layout()
    fig.savefig("results/" + args.out, dpi=110)
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
