"""
Gymnasium environments for the FSAE Driverless dynamic events (Driverless
Supplement 2026, DD.2-DD.4):

    SkidpadEnv        two laps right, two left; timed laps averaged
    AccelerationEnv   75 m from a standing start, then stop
    AutocrossEnv      one lap of a randomly generated course, then stop

All three share `DriverlessEventEnv`, which follows the event procedure:

  * The car is staged from rest per the event rule (DD.4.x staging) with the
    steering straight (DD.3.1.2). Curriculum episodes instead spawn rolling
    part-way through the run; their timing is not valid.
  * Timing lines are crossed by the car's foremost point. A timed segment runs
    from one crossing to another; the run time is the mean of the timed
    segments (one segment for accel and autocross, two laps for skidpad).
  * Corrected time = run time + cone penalties (+ off-course penalties for
    autocross), as in the DD.4.x scoring rules.
  * Off course (DD.2.1.2) is all four wheels outside the lane: a DNF in
    acceleration and skidpad, +10 s per occurrence in autocross.
  * After the finish the car must come to a full stop inside the stop area
    (DD.4.x finish). Overrunning it, or leaving the lane there, is an unsafe
    stop (DQ). A run is complete only once the car has stopped.

The policy sees a simulated cone sensor, not ground truth: `sensor="simple"`
(ConeSensor, the default) or `sensor="lidar_camera"` (LidarCameraSensor, a
LIDAR-camera pipeline with latency, clutter and ego-state noise), or an
instance of either. Timing, penalties and
reward come from true position, as with trackside timing.

Observation (9 + top_k*7,), 65-dim with the default top_k=8:
  [vx, vy, yaw_rate, prev_steer, prev_accel, progress, before_start, finished,
   next_turn,
   then top_k detection slots of [range_norm, bearing_norm, is_blue,
   is_yellow, is_orange, is_large_orange, valid], nearest first]
  progress = timing lines crossed / timing lines in the run.
  before_start and finished are 0/1 flags. The entry and exit lanes look alike
  to the sensor, and the right pedal is opposite in each, so the phase is given
  explicitly (the car's own state machine knows it).
  next_turn is the way the course goes after the next timing line: +1 left,
  -1 right, 0 straight. It only varies in skidpad, where the lap sequence is
  fixed by the rules and every gate approach looks the same; it is 0 elsewhere
  and after the finish.

Action (2,), both in [-1, 1]: a steering and pedal target. How it is applied
depends on `action_mode`:
    "target" (default)  the applied command moves toward the target at most
                        `steer_rate` / `pedal_rate` full-scale units per
                        second, as a servo would. Full-lock chatter cannot
                        reach the car.
    "delta"             the action is a rate: the command changes by
                        action * rate * dt. Smoothest, but slow to learn.
    "absolute"          applied directly, no rate limit.
The observation's prev_steer and prev_accel are the commands actually applied.

Launch assist: after a staged start the pedal is held at full throttle until
the car reaches `launch_assist_speed` (default 3 m/s); the policy steers
throughout and takes over the pedal from then on. At a standstill braking
changes nothing, so a policy that settles on the brake gets no signal that the
throttle would help and stalls on every staged run. Once the car is moving,
braking has consequences it can learn from. Set 0 to disable.

Reward, per step:
    + progress along the centerline (m), until the finish
    - w_time                         a small cost per step, so faster is better
    - w_stop_speed * speed           after the finish, until stopped
    - min(w_edge * excess^2, edge_cap)   excess = |cte| beyond legal_cte
    - w_action * |action|^2 (delta) or w_action * (target - applied)^2
    - rulebook time penalties * penalty_reward_per_s (cones, off course)
Bonuses: per timing line, per timed segment (segment_bonus scaled by how fast it
was, from the event's reference times), and finish_bonus for a completed run.
Rulebook seconds convert at what one second off the scored result is worth on
track (every timed segment's bonus and time cost), unless
`penalty_reward_per_s` is given.
Terminal penalties: stall > unsafe stop > DNF. The edge penalty is capped so
that driving and crashing always scores better than stalling at the start.
Designed for gamma = 0.999, as used in train.py. `reward_scale` multiplies the
whole reward by a constant (train.py uses 0.05); that keeps every term in the
same proportion, unlike running reward normalisation, which rescales as
training goes and clips large terminal rewards.

Not modelled: the 30 s allowed to restart after a standstill (DD.3.3.1). A
stall before the finish ends the episode, with a grace period after a staged
start.

Usage:

    env = AutocrossEnv()                 # new random course every reset
    env = make_env("acceleration", tire=make_tire(grip_scale=0.89))
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step([steer, accel])

`reset(options=...)` accepts "curriculum" (True/False to force a rolling or
staged start), "curriculum_range" (index into track.curriculum_ranges) and, for
autocross, "track_seed".

`info` carries status, corrected_time, run_time, penalty_s, segment_times,
cones_hit, off_courses, crossings, cte, s, and the vehicle diagnostics.
"""
from dataclasses import dataclass

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from vehicle_model import FourCornerVehicle, VehicleParams
from track import SkidpadTrack, AccelerationTrack, AutocrossTrack
from sensor import ConeSensor
from sensor_lidar_camera import LidarCameraSensor

SENSORS = {"simple": ConeSensor, "lidar_camera": LidarCameraSensor}


def make_sensor(sensor):
    """A sensor instance from an instance, a name in SENSORS, or None."""
    if sensor is None:
        return ConeSensor()
    if isinstance(sensor, str):
        try:
            return SENSORS[sensor]()
        except KeyError:
            raise ValueError(f"unknown sensor {sensor!r}; choose from {sorted(SENSORS)}")
    return sensor


@dataclass
class EpisodeLimits:
    """Control rate, episode budget, and the conditions that end an episode."""

    control_dt: float = 0.02              # s, 50 Hz control rate
    time_budget_s: float = None           # s; None uses the event's budget
    spin_yaw_rate_limit: float = 6.0      # rad/s, above this counts as a spin-out
    stall_speed_mps: float = 0.5          # below this counts toward a stall
    stall_time_s: float = 0.75            # consecutive seconds stalled to terminate
    launch_grace_s: float = 2.0           # extra stall allowance before the start line
    stop_speed_mps: float = 0.1           # "full stop" after the finish
    stop_hold_s: float = 0.2              # held this long
    lost_margin_m: float = 3.0            # CG this far outside the lane ends the run
    stall_penalty: float = 150.0          # harsh, so idling is never cheaper than trying
    dnf_penalty: float = 10.0             # off course, lost, spun out
    unsafe_stop_penalty: float = 40.0     # overran the stop area or left the lane there
    steer_rate: float = 5.0               # delta mode: full-scale units/s (lock to lock 0.4 s)
    pedal_rate: float = 8.0               # delta mode: full-scale units/s


# Kept so `from env import CONTROL_DT` still resolves. Read `env.control_dt`.
CONTROL_DT = EpisodeLimits.control_dt

STATUS_RUNNING = "running"
STATUS_COMPLETED = "completed"


class DriverlessEventEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, track=None, track_factory=None, vehicle_params: VehicleParams = None,
                 sensor=None, limits: EpisodeLimits = None, tire=None,
                 action_mode="absolute", w_edge=2.0, edge_cap=0.3, w_action=0.02,
                 w_time=0.02, w_stop_speed=0.005, curriculum_prob=0.5,
                 crossing_bonus=10.0, segment_bonus=100.0, finish_bonus=150.0,
                 penalty_reward_per_s=None, crossing_band_m=5.0, reward_scale=1.0,
                 launch_assist_speed=3.0):
        super().__init__()
        if track is None and track_factory is None:
            raise ValueError("give a track or a track_factory")
        self.vehicle = FourCornerVehicle(vehicle_params, tire=tire)
        self.track_factory = track_factory
        self.track = track if track is not None else track_factory(np.random.default_rng(0))
        self.sensor = make_sensor(sensor)
        self.limits = limits or EpisodeLimits()
        if hasattr(self.sensor, "set_control_dt"):
            self.sensor.set_control_dt(self.limits.control_dt)
        if action_mode not in ("target", "delta", "absolute"):
            raise ValueError("action_mode must be 'target', 'delta' or 'absolute', "
                             f"got {action_mode!r}")
        self.action_mode = action_mode
        self.w_edge = w_edge
        self.edge_cap = edge_cap
        self.w_action = w_action
        self.w_time = w_time
        self.w_stop_speed = w_stop_speed
        self.curriculum_prob = curriculum_prob
        self.crossing_bonus = crossing_bonus
        self.segment_bonus = segment_bonus
        self.finish_bonus = finish_bonus
        self._penalty_rate_arg = penalty_reward_per_s
        self.crossing_band_m = crossing_band_m
        self.reward_scale = reward_scale
        self.launch_assist_speed = launch_assist_speed

        dt = self.limits.control_dt
        self.stall_steps_limit = int(round(self.limits.stall_time_s / dt))
        self.launch_grace_steps = int(round(self.limits.launch_grace_s / dt))
        self.stop_hold_steps = max(1, int(round(self.limits.stop_hold_s / dt)))

        self.top_k = self.sensor.top_k
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        obs_dim = 9 + self.top_k * self.sensor.slot_size
        obs_high = np.full(obs_dim, 2.0, dtype=np.float32)
        obs_high[0:3] = [50, 20, 10]  # vx, vy, yaw_rate can exceed 2
        self.observation_space = spaces.Box(low=-obs_high, high=obs_high, dtype=np.float32)

        self._prev_action = np.zeros(2, dtype=np.float32)
        self._configure_track()
        self.trajectory = []
        self.last_detections = None

    # ---------------- setup ----------------
    @property
    def control_dt(self):
        """Seconds per control step. Read this rather than the module constant."""
        return self.limits.control_dt

    @property
    def rules(self):
        return self.track.rules

    def _configure_track(self):
        """Everything that depends on the current track."""
        p, tr = self.vehicle.p, self.track
        self.legal_cte = tr.legal_cte(p.half_width)
        if self.legal_cte <= 0.0:
            raise ValueError(f"car is too wide for the lane: legal_cte = {self.legal_cte:.3f} m")
        self.front_offset = p.lf + p.front_overhang
        self._fp_front = p.lf + max(p.wheel_radius, p.front_overhang)
        self._fp_rear = p.lr + p.wheel_radius
        self._fp_side = p.half_width
        x_off = np.array([p.lf, p.lf, -p.lr, -p.lr])
        y_off = np.array([p.track_f, -p.track_f, p.track_r, -p.track_r]) / 2.0
        self._wheel_off = (x_off, y_off)
        self.n_crossings = len(tr.crossing_idx)
        self.max_laps = self.n_crossings      # kept for older plotting code
        self._band = int(round(self.crossing_band_m / tr.ds))
        budget = self.limits.time_budget_s or tr.rules.time_budget_s
        self.max_steps = int(budget / self.limits.control_dt)
        # Rulebook seconds to reward: by default a penalty second costs exactly
        # what one second off the scored result costs on track. The result is
        # the mean of the timed segments, so one second off it is one second
        # off every segment: each segment bonus loses segment_bonus/span, and
        # every segment's steps cost w_time per step.
        if self._penalty_rate_arg is not None:
            self.penalty_reward_per_s = self._penalty_rate_arg
        else:
            refs = tr.rules.segment_refs or ((0.0, 1.0),)
            n = len(refs)
            bonus_rate = sum(self.segment_bonus / (slow - fast) for fast, slow in refs)
            time_rate = n * self.w_time / self.limits.control_dt
            self.penalty_reward_per_s = bonus_rate + time_rate
        self._cone_hit = np.zeros(len(tr.cone_xy), dtype=bool)

    def _front_point(self, x, y, yaw):
        return x + self.front_offset * np.cos(yaw), y + self.front_offset * np.sin(yaw)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        if self.track_factory is not None:
            track_seed = options.get("track_seed")
            if track_seed is None:
                track_seed = int(self.np_random.integers(2 ** 31 - 1))
            self.track = self.track_factory(np.random.default_rng(track_seed))
            self._configure_track()
        tr, p = self.track, self.vehicle.p

        curriculum = options.get("curriculum")
        if curriculum is None:
            curriculum = (bool(tr.curriculum_ranges)
                          and self.np_random.random() < self.curriculum_prob)
        if curriculum and tr.curriculum_ranges:
            k = options.get("curriculum_range")
            if k is None:
                k = int(self.np_random.integers(len(tr.curriculum_ranges)))
            lo, hi, v_lo, v_hi = tr.curriculum_ranges[k]
            idx0 = int(self.np_random.integers(lo, hi))
            speed0 = float(self.np_random.uniform(v_lo, v_hi))
            self.staged = False
        else:
            idx0 = tr.staging_idx(p.front_overhang, p.lf, p.wheel_radius)
            speed0 = 0.0
            self.staged = True

        x0, y0 = float(tr.cx[idx0]), float(tr.cy[idx0])
        yaw0 = float(tr.heading[idx0])
        self.vehicle.reset(x=x0, y=y0, yaw=yaw0, vx=speed0)
        self.sensor.reset(len(tr.cone_xy), track=tr, rng=self.np_random)

        # Crossings already behind the car count as passed, without a time.
        fx, fy = self._front_point(x0, y0, yaw0)
        self._next_cross = 0
        for ci in tr.crossing_idx:
            h = tr.heading[ci]
            along = (fx - tr.cx[ci]) * np.cos(h) + (fy - tr.cy[ci]) * np.sin(h)
            if ci <= idx0 or (abs(idx0 - ci) <= self._band and along >= 0.0):
                self._next_cross += 1
            else:
                break
        self._cross_times = [None] * self.n_crossings
        self._prev_front = (fx, fy)

        self._prev_action[:] = 0.0
        self.applied_action = self._prev_action.copy()
        self._steps = 0
        self._low_speed_steps = 0
        self._stopped_steps = 0
        self._assisting = self.staged and self.launch_assist_speed > 0.0
        self._has_moved = speed0 > self.limits.stall_speed_mps
        self._prev_s = tr.s[idx0]
        self._track_idx = idx0
        self._cone_hit[:] = False
        self._off_course = False
        self.off_courses = 0
        self.segment_times = {}
        self.status = STATUS_RUNNING
        self.trajectory = [(x0, y0)]
        return self._get_obs(), {}

    # ---------------- per-step helpers ----------------
    def _detect_cone_hits(self, x, y, yaw):
        """Mark cones whose base overlaps the car footprint. Returns new hits."""
        tr = self.track
        dx = tr.cone_xy[:, 0] - x
        dy = tr.cone_xy[:, 1] - y
        c, s = np.cos(yaw), np.sin(yaw)
        bx = dx * c + dy * s
        by = -dx * s + dy * c
        r = tr.cone_radius
        hit = ((bx <= self._fp_front + r) & (bx >= -self._fp_rear - r)
               & (np.abs(by) <= self._fp_side + r))
        new = hit & ~self._cone_hit
        self._cone_hit |= hit
        return int(new.sum())

    def _all_wheels_outside(self, x, y, yaw, idx):
        """DD.2.1.2: every wheel entirely outside the lane edge."""
        x_off, y_off = self._wheel_off
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x + x_off * c - y_off * s
        wy = y + x_off * s + y_off * c
        lat = self.track.lateral_offset(wx, wy, idx)
        edge = self.track.track_width / 2.0 + self.vehicle.p.tire_width / 2.0
        return bool(np.all(np.abs(lat) > edge))

    def _check_crossing(self, fx, fy, yaw, t_now):
        """Advance the timing if the foremost point crossed the next line."""
        tr = self.track
        k = self._next_cross
        if k >= self.n_crossings:
            return None
        ci = tr.crossing_idx[k]
        if abs(self._track_idx - ci) > self._band:
            return None
        h = tr.heading[ci]
        c, s = np.cos(h), np.sin(h)
        px, py = self._prev_front
        prev_along = (px - tr.cx[ci]) * c + (py - tr.cy[ci]) * s
        along = (fx - tr.cx[ci]) * c + (fy - tr.cy[ci]) * s
        lateral = -(fx - tr.cx[ci]) * s + (fy - tr.cy[ci]) * c
        if not (prev_along < 0.0 <= along):
            return None
        if abs(lateral) > tr.track_width / 2.0 + 0.5 or np.cos(yaw - h) <= 0.0:
            return None
        dt = self.limits.control_dt
        frac = -prev_along / (along - prev_along)
        # Crossings already behind a rolling spawn keep no time, so segments
        # that started before the spawn are never timed.
        self._cross_times[k] = t_now - dt + frac * dt
        self._next_cross += 1
        return k

    def _penalty_seconds(self):
        """Rulebook penalty seconds earned so far in this run."""
        rules = self.rules
        pen = int(self._cone_hit.sum()) * rules.cone_penalty_s
        if rules.off_course_penalty_s is not None:
            pen += self.off_courses * rules.off_course_penalty_s
        return float(pen)

    def _run_result(self):
        """(run_time, penalty_s, corrected_time); None where not available."""
        rules = self.rules
        times = []
        for label, i, j in self.track.timed_segments:
            if label not in self.segment_times:
                times = None
                break
            times.append(self.segment_times[label])
        run_time = None if not times else float(np.mean(times))
        penalty = int(self._cone_hit.sum()) * rules.cone_penalty_s
        if rules.off_course_penalty_s is not None:
            penalty += self.off_courses * rules.off_course_penalty_s
        corrected = None
        if run_time is not None and self.status == STATUS_COMPLETED:
            corrected = run_time + penalty
        return run_time, float(penalty), corrected

    def _get_obs(self):
        x, y, yaw, vx, vy, r, *_ = self.vehicle.state
        tr = self.track
        detections = self.sensor.sense(x, y, yaw, tr.cone_xy, tr.cone_color, self.np_random)
        self.last_detections = detections
        if hasattr(self.sensor, "observe_ego"):
            vx, vy, r = self.sensor.observe_ego(vx, vy, r, self.np_random)
        progress = float(self._next_cross) / float(self.n_crossings)
        before_start = 1.0 if self._next_cross == 0 else 0.0
        finished = 1.0 if self._next_cross >= self.n_crossings else 0.0
        next_turn = (0.0 if finished
                     else float(self.track.crossing_turns[self._next_cross]))
        obs = np.concatenate([
            [vx, vy, r, self._prev_action[0], self._prev_action[1], progress,
             before_start, finished, next_turn],
            detections.flatten(),
        ]).astype(np.float32)
        return obs

    # ---------------- step ----------------
    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        tr, lim, rules = self.track, self.limits, self.rules
        dt = lim.control_dt
        prev = self._prev_action.copy()
        rates = np.array([lim.steer_rate, lim.pedal_rate], dtype=np.float32)
        if self.action_mode == "delta":
            applied = np.clip(prev + action * rates * dt, -1.0, 1.0)
            action_cost = float(np.sum(action ** 2))
        elif self.action_mode == "target":
            step = np.clip(action - prev, -rates * dt, rates * dt)
            applied = np.clip(prev + step, -1.0, 1.0)
            action_cost = float(np.sum((action - prev) ** 2))
        else:
            applied = action
            action_cost = float(np.sum((action - prev) ** 2))
        if self._assisting:
            applied = applied.copy()
            applied[1] = 1.0                    # launch assist: full throttle
        self._prev_action = applied.astype(np.float32)
        self.applied_action = self._prev_action.copy()
        _, vinfo = self.vehicle.step(applied, dt)
        self._steps += 1
        t_now = self._steps * dt

        x, y, yaw, vx, vy, r, *_ = self.vehicle.state
        speed = float(np.hypot(vx, vy))
        if self._assisting and speed >= self.launch_assist_speed:
            self._assisting = False
        self.trajectory.append((x, y))
        fx, fy = self._front_point(x, y, yaw)
        finished_before = self._next_cross >= self.n_crossings

        reward = 0.0
        crossed = self._check_crossing(fx, fy, yaw, t_now)
        self._prev_front = (fx, fy)
        if crossed is not None:
            for seg_no, (label, i, j) in enumerate(tr.timed_segments):
                if j == crossed and self._cross_times[i] is not None \
                        and self._cross_times[j] is not None:
                    seg_t = self._cross_times[j] - self._cross_times[i]
                    self.segment_times[label] = seg_t
                    if seg_no < len(rules.segment_refs):
                        fast, slow = rules.segment_refs[seg_no]
                        q = np.clip((slow - seg_t) / (slow - fast), 0.0, 1.0)
                        reward += self.segment_bonus * float(q)
            if crossed < self.n_crossings - 1:
                reward += self.crossing_bonus
        finished = self._next_cross >= self.n_crossings

        dist, idx, s, cte, _ = tr.nearest_windowed(x, y, self._track_idx)
        d_s = s - self._prev_s
        self._track_idx = idx
        self._prev_s = s

        # --- shaping ---
        if not finished_before:
            reward += d_s
        else:
            reward -= self.w_stop_speed * speed
        edge_excess = max(0.0, abs(cte) - self.legal_cte)
        reward -= min(self.w_edge * edge_excess ** 2, self.edge_cap)
        reward -= self.w_action * action_cost + self.w_time

        new_cones = self._detect_cone_hits(x, y, yaw)
        reward -= new_cones * rules.cone_penalty_s * self.penalty_reward_per_s

        # --- run status ---
        status = STATUS_RUNNING
        outside = self._all_wheels_outside(x, y, yaw, idx)
        new_off_course = outside and not self._off_course
        self._off_course = outside

        if speed > lim.stall_speed_mps:
            self._has_moved = True
        if abs(r) > lim.spin_yaw_rate_limit:
            status = "dnf_spin"
        elif dist > tr.track_width / 2.0 + lim.lost_margin_m:
            status = "unsafe_stop" if finished else "dnf_lost"
        elif finished:
            stop_limit = tr.s[tr.finish_idx] + rules.stop_distance_m
            if outside or s + self.front_offset > stop_limit:
                status = "unsafe_stop"
            else:
                self._stopped_steps = self._stopped_steps + 1 if speed < lim.stop_speed_mps else 0
                if self._stopped_steps >= self.stop_hold_steps:
                    status = STATUS_COMPLETED
        else:
            if new_off_course:
                if rules.off_course_penalty_s is None:
                    status = "dnf_off_course"
                else:
                    self.off_courses += 1
                    reward -= rules.off_course_penalty_s * self.penalty_reward_per_s
            if status == STATUS_RUNNING:
                self._low_speed_steps = self._low_speed_steps + 1 if vx < lim.stall_speed_mps else 0
                # The launch allowance lasts until the start line, so trying
                # the throttle and backing off is never punished sooner than
                # never moving.
                limit = self.stall_steps_limit
                if self._next_cross == 0:
                    limit += self.launch_grace_steps
                if self._low_speed_steps >= limit:
                    status = "dnf_stall"

        self.status = status
        if status == STATUS_COMPLETED:
            reward += self.finish_bonus
        elif status == "dnf_stall":
            reward -= lim.stall_penalty
        elif status == "unsafe_stop":
            reward -= lim.unsafe_stop_penalty
        elif status != STATUS_RUNNING:
            reward -= lim.dnf_penalty

        terminated = status != STATUS_RUNNING
        truncated = (not terminated) and self._steps >= self.max_steps
        if truncated:
            self.status = "timeout"
        run_time, penalty_s, corrected = self._run_result()
        obs = self._get_obs()
        return obs, float(reward * self.reward_scale), terminated, truncated, {
            "status": self.status, "finished": self.status == STATUS_COMPLETED,
            "stalled": self.status == "dnf_stall", "spun_out": self.status == "dnf_spin",
            "off_track": self.status in ("dnf_off_course", "dnf_lost"),
            "crossings": self._next_cross, "segment_times": dict(self.segment_times),
            "cones_hit": int(self._cone_hit.sum()), "off_courses": self.off_courses,
            "run_time": run_time, "penalty_s": penalty_s, "corrected_time": corrected,
            "cte": cte, "s": s, **vinfo,
        }


class SkidpadEnv(DriverlessEventEnv):
    def __init__(self, track=None, crossing_start_prob=None, **kwargs):
        if crossing_start_prob is not None:
            kwargs["curriculum_prob"] = crossing_start_prob
        super().__init__(track=track or SkidpadTrack(), **kwargs)


class AccelerationEnv(DriverlessEventEnv):
    def __init__(self, track=None, **kwargs):
        super().__init__(track=track or AccelerationTrack(), **kwargs)


class AutocrossEnv(DriverlessEventEnv):
    """New random course every reset unless a fixed `track` is given.
    `track_kwargs` go to AutocrossTrack; `reset(options={"track_seed": n})`
    picks a specific course."""

    def __init__(self, track=None, track_kwargs=None, **kwargs):
        track_kwargs = dict(track_kwargs or {})
        factory = None
        if track is None:
            def factory(rng):
                return AutocrossTrack(seed=int(rng.integers(2 ** 31 - 1)), **track_kwargs)
        super().__init__(track=track, track_factory=factory, **kwargs)


ENVS = {"skidpad": SkidpadEnv, "acceleration": AccelerationEnv,
        "accel": AccelerationEnv, "autocross": AutocrossEnv}


def make_env(event, **kwargs):
    try:
        cls = ENVS[event.lower()]
    except KeyError:
        raise ValueError(f"unknown event {event!r}; choose from {sorted(set(ENVS))}")
    return cls(**kwargs)
