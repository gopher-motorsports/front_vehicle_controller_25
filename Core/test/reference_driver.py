"""
Ground-truth reference driver for checking the environments and the car.

It is not a policy: it reads the env's true state, centerline and timing
progress, which the RL policy never sees. Use it to confirm timing, penalties
and stopping work, and as a baseline for what a competent driver gets out of the
car model on each event.

Steering is pure pursuit on the centerline, offset toward the inside of corners
in proportion to the (smoothed) curvature. Speed follows a profile: a corner
speed from `ay_max`, a braking limit `ax_brake` looking ahead, a cap `speed`,
and a stop inside the stop area after the finish.

Usage:

    env = make_env("autocross", curriculum_prob=0.0)
    driver = ReferenceDriver(env)
    res = rollout(env, driver)            # from evaluate.py
"""
import numpy as np


class ReferenceDriver:
    def __init__(self, env, speed=30.0, ay_max=8.5, ax_brake=6.0, stop_decel=4.0,
                 line_offset=None, full_offset_radius=10.0, smooth_m=8.0, stop_fraction=0.7,
                 lookahead_s=0.3, min_lookahead_m=2.0, k_speed=0.6, ki_speed=0.4):
        self.env = env
        self.speed = speed
        self.ay_max = ay_max
        self.ax_brake = ax_brake
        self.stop_decel = stop_decel
        # Default: part of the way to the legal line. Pure pursuit cuts inside
        # its target and the rear tires track inside the fronts.
        self.line_offset = line_offset
        self.full_offset_radius = full_offset_radius
        self.smooth_m = smooth_m
        self.stop_fraction = stop_fraction
        self.lookahead_s = lookahead_s
        self.min_lookahead_m = min_lookahead_m
        self.k_speed = k_speed
        self.ki_speed = ki_speed
        self._track = None
        self._integ = 0.0

    def _prepare(self):
        env, tr = self.env, self.env.track
        self._track = tr
        n = max(1, int(round(self.smooth_m / tr.ds)))
        k = np.convolve(tr.curvature, np.ones(n) / n, mode="same")
        offset = self.line_offset if self.line_offset is not None else 0.6 * env.legal_cte
        self._offset = np.sign(k) * offset * np.minimum(1.0, np.abs(k) * self.full_offset_radius)

        # Corner speed from the tightest curvature nearby, so the car is
        # already slow at the corner entry rather than part-way in.
        k_abs = np.abs(tr.curvature)
        k_max = np.array([k_abs[max(0, i - n):i + n + 1].max() for i in range(len(k_abs))])
        v = np.minimum(self.speed, np.sqrt(self.ay_max / np.maximum(k_max, 1e-4)))
        for i in range(len(v) - 2, -1, -1):          # braking limit, backwards
            ds = tr.s[i + 1] - tr.s[i]
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * self.ax_brake * ds))
        # Gentle stop, finished well inside the stop area.
        stop_s = tr.s[tr.finish_idx] + self.stop_fraction * tr.rules.stop_distance_m
        v = np.minimum(v, np.sqrt(2 * self.stop_decel * np.maximum(stop_s - tr.s, 0.0)))
        self._v = v

    def __call__(self, obs):
        env = self.env
        if env.track is not self._track:
            self._prepare()
        tr, p = env.track, env.vehicle.p
        x, y, yaw, vx, vy, *_ = env.vehicle.state
        speed = float(np.hypot(vx, vy))
        if env._steps == 0:
            self._integ = 0.0

        ld = max(self.min_lookahead_m, self.lookahead_s * speed)
        i = min(env._track_idx + int(ld / tr.ds), len(tr.cx) - 1)
        h = tr.heading[i]
        o = self._offset[i]
        tx = tr.cx[i] - np.sin(h) * o
        ty = tr.cy[i] + np.cos(h) * o
        dx, dy = tx - x, ty - y
        by = -dx * np.sin(yaw) + dy * np.cos(yaw)
        curvature = 2.0 * by / max(dx * dx + dy * dy, 1e-6)
        steer = float(np.clip(np.arctan(p.wheelbase * curvature) / p.max_steer_angle, -1, 1))

        # Speed target a little ahead, to allow for the controller's lag.
        j = min(env._track_idx + int(0.25 * speed / tr.ds), len(tr.cx) - 1)
        v_target = self._v[j]
        if v_target <= 0.0 and env._next_cross >= env.n_crossings:
            return self._emit(steer, -1.0)                      # stop and hold
        err = v_target - speed
        self._integ = float(np.clip(self._integ + err * env.control_dt, -2.0, 2.0))
        if err < -0.5:
            self._integ = min(self._integ, 0.0)
        accel = float(np.clip(self.k_speed * err + self.ki_speed * self._integ, -1.0, 1.0))
        return self._emit(steer, accel)

    def _emit(self, steer, accel):
        """Absolute commands, converted to rate commands in delta mode."""
        env = self.env
        target = np.array([steer, accel], dtype=np.float32)
        if getattr(env, "action_mode", "absolute") != "delta":
            return target            # target and absolute modes take the command itself
        lim = env.limits
        rates = np.array([lim.steer_rate, lim.pedal_rate], dtype=np.float32)
        return np.clip((target - env.applied_action) / (rates * env.control_dt), -1.0, 1.0)
