"""
Simulated cone detector: a forward-facing range and bearing sensor standing in
for a stereo camera or LiDAR.

Returns a fixed-size, nearest-first, zero-padded detection list so a varying
number of visible cones still gives the policy a fixed-length observation.

Models a field of view and maximum range, Gaussian noise on range and bearing,
and dropout that both rises with range and persists across frames. Persistence
is a two-state Markov chain per cone: a cone seen last frame is less likely to
be missed now, one missed last frame is more likely to stay missed. That
produces short runs of hits and misses rather than single-frame flicker.

Not modeled: one cone occluding another, false positives, motion blur, lighting.

All parameters are engineering placeholders, not measured from hardware.

Usage:

    sensor = ConeSensor(fov_deg=120.0, max_range=20.0, top_k=8)
    sensor.reset(len(track.cone_xy))            # once per episode
    det = sensor.sense(x, y, yaw, track.cone_xy, track.cone_color, rng)
    # det is (top_k, 7): [range_norm, bearing_norm, is_blue, is_yellow,
    #                     is_orange, is_large_orange, valid]

Colour codes follow track_base: 0 blue, 1 yellow, 2 small orange, 3 large
orange (DD.1.1). Large orange cones mark timing lines, which is how a real car
knows where the finish is.
"""
import numpy as np


class ConeSensor:
    slot_size = 7
    n_classes = 4
    def __init__(self, fov_deg=120.0, max_range=20.0,
                 range_noise_std=0.08, bearing_noise_std_deg=1.0,
                 near_miss_prob=0.01, far_miss_prob=0.35,
                 persistence_factor=0.4, occlusion_factor=1.8,
                 top_k=8):
        self.fov = np.radians(fov_deg)
        self.max_range = max_range
        self.range_noise_std = range_noise_std
        self.bearing_noise_std = np.radians(bearing_noise_std_deg)
        self.near_miss_prob = near_miss_prob
        self.far_miss_prob = far_miss_prob
        self.persistence_factor = persistence_factor
        self.occlusion_factor = occlusion_factor
        self.top_k = top_k
        self._prev_hit = None

    def reset(self, n_cones, track=None, rng=None):
        """Once per episode. `track` and `rng` are accepted for interface
        compatibility with LidarCameraSensor and unused here."""
        self._prev_hit = np.zeros(n_cones, dtype=bool)
        self.last_truth = np.zeros(0, dtype=int)

    def sense(self, veh_x, veh_y, veh_yaw, cone_xy, cone_color, rng: np.random.Generator):
        """One sensor frame.

        cone_xy is (N,2) world-frame positions, cone_color is (N,) colour
        codes 0-3. Returns (top_k, 7), sorted by ascending noisy range and
        zero-padded when fewer than top_k cones are detected.
        """
        dx = cone_xy[:, 0] - veh_x
        dy = cone_xy[:, 1] - veh_y
        true_range = np.hypot(dx, dy)
        bearing = _wrap(np.arctan2(dy, dx) - veh_yaw)

        in_fov = (np.abs(bearing) <= self.fov / 2) & (true_range <= self.max_range)

        frac = np.clip(true_range / self.max_range, 0.0, 1.0)
        base_miss = self.near_miss_prob + frac * (self.far_miss_prob - self.near_miss_prob)
        miss_prob = np.where(
            self._prev_hit,
            base_miss * self.persistence_factor,
            base_miss * self.occlusion_factor,
        )
        miss_prob = np.clip(miss_prob, 0.0, 0.98)

        detected = in_fov & (rng.random(len(true_range)) > miss_prob)
        self._prev_hit = detected  # feeds next frame's persistence term

        noisy_range = true_range + rng.normal(0.0, self.range_noise_std, size=len(true_range))
        noisy_bearing = bearing + rng.normal(0.0, self.bearing_noise_std, size=len(true_range))

        idx = np.where(detected)[0]
        idx = idx[np.argsort(noisy_range[idx])][: self.top_k]
        self.last_truth = idx          # cone index per output slot, for diagnostics

        out = np.zeros((self.top_k, self.slot_size), dtype=np.float32)
        for slot, i in enumerate(idx):
            out[slot, 0] = np.clip(noisy_range[i] / self.max_range, 0.0, 1.5)
            out[slot, 1] = np.clip(noisy_bearing[i] / (self.fov / 2), -1.5, 1.5)
            out[slot, 2 + int(cone_color[i])] = 1.0
            out[slot, 6] = 1.0
        return out


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
