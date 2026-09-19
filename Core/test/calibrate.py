"""
Calibrate the tire grip derate against a measured skidpad time.

The timed skidpad lap is the second lap on a circle, so it is close to steady
state. This script finds the fastest speed the full vehicle model can hold on a
constant-radius line (the tight legal line by default), turns that into a
flying-lap time, and, given a measured adjusted time, bisects the tire output
derate until the two match.

Calibrate against your own car: a clean (no cones) adjusted time on a surface
like the one you want to model, with the same tire, pressure and setup as the
model. Fit the derate once, then use `--grip-scale` in train.py and
evaluate.py. Do not tune the derate to hit a leaderboard target: the point is a
model you can trust, then use it to explore setup changes.

Usage:
    python calibrate.py                                   # current model
    python calibrate.py --measured-time 5.31              # fit SimpleTire derate
    python calibrate.py --tire-npz HSR.npz --measured-time 5.31
"""
import argparse
import numpy as np

from vehicle_model import FourCornerVehicle, VehicleParams
from track import Figure8Track
from tire import make_tire, OUTPUT_SCALE

G = 9.81


def _holds_speed(vehicle, radius, v_target, sim_time=12.0, settle_time=5.0, dt=0.02):
    """Drive a constant-radius circle at v_target. True if the car holds both
    the radius (within 0.5 m) and the speed over the final `settle_time`."""
    p = vehicle.p
    vehicle.reset(x=radius, y=0.0, yaw=np.pi / 2, vx=v_target)
    integ = 0.0
    tail = []
    n = int(sim_time / dt)
    ff = np.arctan(p.wheelbase / radius) / p.max_steer_angle
    for k in range(n):
        x, y, yaw, vx, vy, r, *_ = vehicle.state
        e = np.hypot(x, y) - radius
        tangent = np.arctan2(y, x) + np.pi / 2
        course_err = (yaw + np.arctan2(vy, vx) - tangent + np.pi) % (2 * np.pi) - np.pi
        speed = np.hypot(vx, vy)
        steer = np.clip(ff + 0.6 * e - 1.5 * course_err - 0.3 * (r - speed / radius), -1, 1)
        integ += (v_target - speed) * dt
        accel = np.clip(1.0 * (v_target - speed) + 0.5 * integ, -1, 1)
        vehicle.step([steer, accel], dt)
        if abs(r) > 6.0 or abs(e) > 1.5 or not np.isfinite(speed):
            return False
        if k * dt >= sim_time - settle_time:
            tail.append((abs(e), speed))
    tail = np.array(tail)
    return tail[:, 0].max() < 0.5 and tail[:, 1].mean() > v_target - 0.3


def max_steady_speed(make_vehicle, radius, v_lo=4.0, v_hi=18.0, iters=11):
    """Bisect for the highest speed the car can hold on this radius."""
    vehicle = make_vehicle()
    if not _holds_speed(vehicle, radius, v_lo):
        return float("nan")
    for _ in range(iters):
        mid = 0.5 * (v_lo + v_hi)
        if _holds_speed(vehicle, radius, mid):
            v_lo = mid
        else:
            v_hi = mid
    return v_lo


def flying_lap_time(make_vehicle, radius):
    v = max_steady_speed(make_vehicle, radius)
    return 2 * np.pi * radius / v, v


def lateral_g_for(lap_time, radius):
    """Steady lateral acceleration (g) needed to lap this radius in lap_time."""
    return 4 * np.pi ** 2 * radius / lap_time ** 2 / G


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tire-npz", type=str, default=None)
    ap.add_argument("--matlab-compat", action="store_true")
    ap.add_argument("--grip-scale", type=float, default=None,
                    help="Derate to evaluate (default: the tire's own)")
    ap.add_argument("--measured-time", type=float, default=None,
                    help="Your car's clean adjusted skidpad time (s), to fit the derate")
    ap.add_argument("--line-radius", type=float, default=None,
                    help="Radius of the driven line (m). Default: tight legal line")
    ap.add_argument("--scale-lo", type=float, default=0.3)
    ap.add_argument("--scale-hi", type=float, default=1.5)
    args = ap.parse_args()

    params = VehicleParams()
    track = Figure8Track()
    R = track.circle_diameter / 2.0
    radius = args.line_radius or (R - track.legal_cte(params.half_width))

    def factory(scale):
        def make():
            tire = make_tire(args.tire_npz, scale, params=params,
                             matlab_compat=args.matlab_compat)
            return FourCornerVehicle(params, tire=tire)
        return make

    base_scale = args.grip_scale
    if base_scale is None:
        base_scale = OUTPUT_SCALE
    t, v = flying_lap_time(factory(base_scale), radius)
    print(f"Line radius {radius:.3f} m (centerline {R:.3f} m)")
    print(f"Tire: {'MF96 ' + args.tire_npz if args.tire_npz else 'SimpleTire'}, "
          f"grip scale {base_scale:.3f}")
    print(f"  max steady speed {v:.2f} m/s, {v ** 2 / radius / G:.2f} g, "
          f"flying lap {t:.3f} s")

    if args.measured_time is not None:
        target = args.measured_time
        lo, hi = args.scale_lo, args.scale_hi
        t_lo, _ = flying_lap_time(factory(lo), radius)
        t_hi, _ = flying_lap_time(factory(hi), radius)
        if not (t_hi <= target <= t_lo):
            raise SystemExit(f"measured {target:.3f} s is outside what scales "
                             f"{lo}-{hi} produce ({t_hi:.3f}-{t_lo:.3f} s); "
                             "widen --scale-lo/--scale-hi or check the model")
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            t_mid, _ = flying_lap_time(factory(mid), radius)
            if t_mid > target:
                lo = mid     # too slow: needs more grip
            else:
                hi = mid
        fitted = 0.5 * (lo + hi)
        t_fit, v_fit = flying_lap_time(factory(fitted), radius)
        print(f"\nFitted grip scale {fitted:.3f} "
              f"(model {t_fit:.3f} s vs measured {target:.3f} s)")
        print(f"Use it with: python train.py --grip-scale {fitted:.3f}"
              + (f" --tire-npz {args.tire_npz}" if args.tire_npz else ""))


if __name__ == "__main__":
    main()
