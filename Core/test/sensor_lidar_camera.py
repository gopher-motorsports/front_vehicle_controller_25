"""
LIDAR-camera cone perception, modelled on the pipeline of BIT's Smart Shark I:
H. Tian, J. Ni, J. Hu, "Autonomous Driving System Design for Formula Student
Driverless Racecar", arXiv:1809.07636 (2018).

A statistical model of each stage, not a reimplementation. There are no point
clouds or images; each stage is reduced to the effect that limits detection.

LIDAR (paper III.A)
  * 16 beams, 2 deg apart (the VLP-16 pattern), one scan per 1/scan_rate_hz.
    A beam returns points from an object only where its height at that range is
    above the ground-removal threshold and below the object's top, so gaps
    between beams leave dead zones (paper Fig. 5). Scan-to-scan pitch vibration
    moves those zones around.
  * Points per beam follow the object's width at that height and the horizontal
    angular resolution, with random point dropout.
  * Ground removal (RANSAC plane fit, eq. 1-4) is the height threshold
    `ground_threshold`: returns below it are discarded with the ground.
  * Points are accumulated over the last `accumulate_frames` scans before
    Euclidean clustering (III.A.2-3). An object becomes a cluster once the
    accumulated count reaches `min_cluster_points`. Approaching a cone sweeps
    it through several beams, which is why accumulation extends range.
  * Region of interest: the paper uses a 20 m x 20 m box. Read here as
    +-10 m around the sensor, so nothing beyond 10 m ahead is reported.
    Widen `roi_forward` for a longer-range setup.

Camera (III.B)
  * Every LIDAR cluster is projected into the image and checked by the HOG/SVM
    classifier. Clusters outside the camera's field of view are dropped. Real
    cones are accepted with a probability that falls with range. Clutter is
    accepted at `false_accept`.
  * Clutter: cone-sized objects beside the course (tyre stacks in the paper)
    that pass the LIDAR stage. They are placed at random each episode, clear of
    the lane.
  * Colour comes from HSV and k-means (eq. 10-14). The wrong class is reported
    with a probability that grows with range. Large and small orange cones are
    told apart by size, with their own confusion rate.

Localisation and timing (III.A.2, IV)
  * Positions come from accumulated points placed with LIDAR and GPS-INS
    odometry: per-object noise growing with range, plus an offset shared by the
    whole scan (odometry error).
  * Detections are published `latency_s` after the scan. Between scans the last
    list is reused. It is moved into the car's current frame when
    `motion_compensate` is on (the paper's odometry makes this possible),
    otherwise it stays in the frame of the scan.

Ego state (II.B, Figs. 13-16)
  * `observe_ego` adds GPS-INS and wheel-speed noise to vx, vy (through the
    sideslip estimate) and yaw rate before the policy sees them.

Not modelled: one object occluding another, the car body blocking beams,
rolling shutter and lighting, GPS dropouts.

The paper gives the LIDAR layer count and spacing, the scan and odometry rate
(10 Hz), the ROI box and the cone height. Every other number here is a
placeholder. The paper's cone colours (red, blue, yellow) are not used; classes
follow the 2026 supplement.

Output is the same as ConeSensor: (top_k, 7) rows of
[range_norm, bearing_norm, is_blue, is_yellow, is_orange, is_large_orange,
valid], nearest first, measured from the CG. bearing_norm is bearing / pi,
since motion-compensated detections can end up beside or behind the car.

Usage:

    env = make_env("autocross", sensor="lidar_camera")
    env = make_env("autocross", sensor=LidarCameraSensor(roi_forward=20.0))
"""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree

from track_base import (COLOR_BLUE, COLOR_YELLOW, COLOR_ORANGE, COLOR_ORANGE_LARGE,
                        SMALL_CONE_BASE, LARGE_CONE_BASE)

# DD.1.3.2 cone heights.
SMALL_CONE_HEIGHT = 0.325
LARGE_CONE_HEIGHT = 0.505
# 16 layers, 2 deg apart (paper III.A.2).
VLP16_BEAMS_DEG = np.arange(-15.0, 16.0, 2.0)


class LidarCameraSensor:
    slot_size = 7
    n_classes = 4

    def __init__(self, top_k=8, control_dt=0.02,
                 # --- LIDAR ---
                 beam_angles_deg=VLP16_BEAMS_DEG, mount_height=0.30, mount_x=1.2,
                 mount_pitch_deg=0.0, pitch_noise_deg=0.4, h_res_deg=0.2,
                 point_return_prob=0.9, scan_rate_hz=10.0, horizontal_fov_deg=180.0,
                 roi_forward=10.0, roi_half_width=10.0, ground_threshold=0.06,
                 accumulate_frames=3, min_cluster_points=4,
                 # --- camera ---
                 camera_fov_deg=90.0, camera_full_range=8.0, camera_max_range=15.0,
                 verify_near=0.97, verify_far=0.4, false_accept=0.05,
                 color_error_near=0.01, color_error_per_m=0.004, size_confusion=0.05,
                 # --- localisation and timing ---
                 latency_s=0.1, position_noise=0.03, position_noise_per_m=0.008,
                 odometry_noise=0.03, motion_compensate=True,
                 # --- clutter ---
                 clutter_per_100m=1.0, clutter_offset=(1.0, 4.0),
                 clutter_height=(0.3, 0.5), clutter_width=(0.3, 0.6),
                 # --- ego state ---
                 ego_noise=True, vx_noise=0.05, yaw_rate_noise=0.02,
                 sideslip_noise=0.005,
                 norm_range=20.0):
        self.top_k = top_k
        self.control_dt = control_dt
        self.beams = np.radians(np.asarray(beam_angles_deg, dtype=float))
        self.mount_height = mount_height
        self.mount_x = mount_x
        self.mount_pitch = np.radians(mount_pitch_deg)
        self.pitch_noise = np.radians(pitch_noise_deg)
        self.h_res = np.radians(h_res_deg)
        self.point_return_prob = point_return_prob
        self.scan_period = 1.0 / scan_rate_hz
        self.half_fov = np.radians(horizontal_fov_deg) / 2.0
        self.roi_forward = roi_forward
        self.roi_half_width = roi_half_width
        self.ground_threshold = ground_threshold
        self.accumulate_frames = accumulate_frames
        self.min_cluster_points = min_cluster_points
        self.camera_half_fov = np.radians(camera_fov_deg) / 2.0
        self.camera_full_range = camera_full_range
        self.camera_max_range = camera_max_range
        self.verify_near = verify_near
        self.verify_far = verify_far
        self.false_accept = false_accept
        self.color_error_near = color_error_near
        self.color_error_per_m = color_error_per_m
        self.size_confusion = size_confusion
        self.latency_s = latency_s
        self.position_noise = position_noise
        self.position_noise_per_m = position_noise_per_m
        self.odometry_noise = odometry_noise
        self.motion_compensate = motion_compensate
        self.clutter_per_100m = clutter_per_100m
        self.clutter_offset = clutter_offset
        self.clutter_height = clutter_height
        self.clutter_width = clutter_width
        self.ego_noise = ego_noise
        self.vx_noise = vx_noise
        self.yaw_rate_noise = yaw_rate_noise
        self.sideslip_noise = sideslip_noise
        self.norm_range = norm_range
        self.reset(0)

    # ---------------- episode setup ----------------
    def set_control_dt(self, dt):
        self.control_dt = dt

    def reset(self, n_cones, track=None, rng=None):
        self._n_cones = n_cones
        self._calls = 0
        self._next_scan = 0.0
        self._pending = deque()
        self._published = None
        self._hist = deque(maxlen=self.accumulate_frames)
        self._objects_ready = False
        self._clutter_xy = np.zeros((0, 2))
        if track is not None and self.clutter_per_100m > 0:
            rng = rng if rng is not None else np.random.default_rng()
            self._clutter_xy = self._place_clutter(track, rng)
        n_cl = len(self._clutter_xy)
        rng = rng if rng is not None else np.random.default_rng()
        self._clutter_h = rng.uniform(*self.clutter_height, n_cl)
        self._clutter_w = rng.uniform(*self.clutter_width, n_cl)
        self.last_truth = np.zeros(0, dtype=int)
        self.last_age = None

    def _place_clutter(self, track, rng):
        """Cone-sized objects beside the course, clear of every lane."""
        length = track.s[track.finish_idx] - track.s[track.start_idx]
        n = rng.poisson(self.clutter_per_100m * max(length, 1.0) / 100.0)
        if n == 0:
            return np.zeros((0, 2))
        tree = cKDTree(np.column_stack([track.cx, track.cy]))
        keep_out = track.track_width / 2.0 + 0.5
        out = []
        for _ in range(20 * n):
            if len(out) >= n:
                break
            i = int(rng.integers(track.start_idx, track.finish_idx + 1))
            side = rng.choice([-1.0, 1.0])
            off = side * (track.track_width / 2.0 + rng.uniform(*self.clutter_offset))
            h = track.heading[i]
            p = np.array([track.cx[i] - np.sin(h) * off, track.cy[i] + np.cos(h) * off])
            if tree.query(p)[0] > keep_out:
                out.append(p)
        return np.array(out).reshape(-1, 2)

    def _build_objects(self, cone_xy, cone_color):
        is_large = cone_color == COLOR_ORANGE_LARGE
        self._obj_height = np.concatenate([
            np.where(is_large, LARGE_CONE_HEIGHT, SMALL_CONE_HEIGHT), self._clutter_h])
        self._obj_base = np.concatenate([
            np.where(is_large, LARGE_CONE_BASE, SMALL_CONE_BASE), self._clutter_w])
        self._obj_is_cone = np.concatenate([
            np.ones(len(cone_xy), dtype=bool), np.zeros(len(self._clutter_xy), dtype=bool)])
        self._objects_ready = True

    # ---------------- ego state ----------------
    def observe_ego(self, vx, vy, r, rng):
        if not self.ego_noise:
            return vx, vy, r
        speed = float(np.hypot(vx, vy))
        vy_sigma = max(0.02, speed * self.sideslip_noise)
        return (vx + rng.normal(0.0, self.vx_noise),
                vy + rng.normal(0.0, vy_sigma),
                r + rng.normal(0.0, self.yaw_rate_noise))

    # ---------------- per-step ----------------
    def sense(self, veh_x, veh_y, veh_yaw, cone_xy, cone_color, rng):
        t = self._calls * self.control_dt
        self._calls += 1
        if not self._objects_ready:
            self._build_objects(cone_xy, cone_color)
        if t >= self._next_scan - 1e-9:
            self._scan(t, veh_x, veh_y, veh_yaw, cone_xy, cone_color, rng)
            self._next_scan += self.scan_period
        while self._pending and self._pending[0]["publish_t"] <= t + 1e-9:
            self._published = self._pending.popleft()
        return self._format(t, veh_x, veh_y, veh_yaw)

    def _scan(self, t, x, y, yaw, cone_xy, cone_color, rng):
        obj_xy = np.vstack([cone_xy, self._clutter_xy])
        c, s = np.cos(yaw), np.sin(yaw)
        lx, ly = x + self.mount_x * c, y + self.mount_x * s
        dx, dy = obj_xy[:, 0] - lx, obj_xy[:, 1] - ly
        bx = dx * c + dy * s
        by = -dx * s + dy * c
        d = np.maximum(np.hypot(bx, by), 0.05)
        bearing = np.arctan2(by, bx)
        in_view = ((np.abs(bearing) <= self.half_fov) & (bx >= 0.0)
                   & (bx <= self.roi_forward) & (np.abs(by) <= self.roi_half_width))

        # Beam heights at each object's range, with this scan's pitch.
        pitch = self.mount_pitch + rng.normal(0.0, self.pitch_noise)
        z = self.mount_height + d[:, None] * np.tan(self.beams[None, :] + pitch)
        H = self._obj_height[:, None]
        hit = (z >= self.ground_threshold) & (z <= H)
        taper = np.clip(1.0 - z / H, 0.15, 1.0)
        width = np.where(self._obj_is_cone[:, None], self._obj_base[:, None] * taper,
                         self._obj_base[:, None])
        expected = np.where(hit, width / (d[:, None] * self.h_res), 0.0)
        points = rng.poisson(expected * self.point_return_prob).sum(axis=1)
        points = np.where(in_view, points, 0)
        self._hist.append(points)
        accumulated = np.sum(self._hist, axis=0)
        clustered = in_view & (accumulated >= self.min_cluster_points)

        # Camera check.
        in_camera = (np.abs(bearing) <= self.camera_half_fov) & (d <= self.camera_max_range)
        frac = np.clip((d - self.camera_full_range)
                       / (self.camera_max_range - self.camera_full_range), 0.0, 1.0)
        p_accept = np.where(self._obj_is_cone,
                            self.verify_near + frac * (self.verify_far - self.verify_near),
                            self.false_accept)
        detected = clustered & in_camera & (rng.random(len(d)) < p_accept)
        ids = np.flatnonzero(detected)

        # Class, with range-dependent colour errors and size confusion.
        n_cones = len(cone_xy)
        classes = np.empty(len(ids), dtype=int)
        for k, i in enumerate(ids):
            if i >= n_cones:
                classes[k] = int(rng.integers(0, 3))      # clutter passed as a cone
                continue
            true_c = int(cone_color[i])
            p_err = self.color_error_near + self.color_error_per_m * d[i]
            cls = true_c
            if true_c == COLOR_ORANGE_LARGE:
                if rng.random() < self.size_confusion:
                    cls = COLOR_ORANGE
            elif rng.random() < p_err:
                cls = int(rng.choice([k2 for k2 in (COLOR_BLUE, COLOR_YELLOW, COLOR_ORANGE)
                                      if k2 != true_c]))
            elif true_c == COLOR_ORANGE and rng.random() < self.size_confusion:
                cls = COLOR_ORANGE_LARGE
            classes[k] = cls

        # Positions with range noise and a shared odometry offset.
        sigma = self.position_noise + self.position_noise_per_m * d[ids]
        est = (obj_xy[ids] + rng.normal(0.0, 1.0, (len(ids), 2)) * sigma[:, None]
               + rng.normal(0.0, self.odometry_noise, 2))
        self._pending.append({
            "publish_t": t + self.latency_s, "scan_t": t, "pose": (x, y, yaw),
            "xy": est, "classes": classes, "ids": ids,
        })

    def _format(self, t, x, y, yaw):
        out = np.zeros((self.top_k, self.slot_size), dtype=np.float32)
        pub = self._published
        if pub is None or len(pub["ids"]) == 0:
            self.last_truth = np.zeros(0, dtype=int)
            self.last_age = None if pub is None else t - pub["scan_t"]
            return out
        rx, ry, ryaw = (x, y, yaw) if self.motion_compensate else pub["pose"]
        c, s = np.cos(ryaw), np.sin(ryaw)
        dx, dy = pub["xy"][:, 0] - rx, pub["xy"][:, 1] - ry
        bx = dx * c + dy * s
        by = -dx * s + dy * c
        rng_ = np.hypot(bx, by)
        order = np.argsort(rng_)[: self.top_k]
        for slot, j in enumerate(order):
            out[slot, 0] = np.clip(rng_[j] / self.norm_range, 0.0, 1.5)
            out[slot, 1] = np.arctan2(by[j], bx[j]) / np.pi
            out[slot, 2 + int(pub["classes"][j])] = 1.0
            out[slot, 6] = 1.0
        self.last_truth = pub["ids"][order]
        self.last_age = t - pub["scan_t"]
        return out
