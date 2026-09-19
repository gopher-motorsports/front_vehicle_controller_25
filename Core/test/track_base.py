"""
Shared pieces for every event track: the unrolled centerline, cone placement,
timing lines, and the event rules from the FSAE Driverless Supplement 2026
(Version 1.0, 29 Sept 2025).

Every track is stored as one UNROLLED centerline: staging area, then the whole
run in driving order, then the stop area. Laps that retrace the same ground
(skidpad circles, the autocross stop area running onto the start of the lap)
get their own indices, so the index only moves forward as the car drives.
Always track the car with `nearest_windowed`, never a global search.

A track provides:
    cx, cy, heading, s, curvature   centerline arrays
    track_width                     lane width (m)
    cone_xy, cone_color, cone_radius   (N,2), (N,), (N,)
    crossing_idx                    centerline index of each timing-line
                                    crossing, in the order they are driven
    timed_segments                  [(label, i, j)]: time from crossing i to j
    curriculum_ranges               [(lo, hi, v_lo, v_hi)]: index range and
                                    speed range (m/s) for rolling spawns
    crossing_turns                  per crossing, the way the course goes
                                    after it: +1 left, -1 right, 0 straight
    rules                           EventRules

Cone colours follow DD.1.1: blue left border, yellow right border, small orange
for entry and exit lanes, large orange before and after timing lines.
"""
from dataclasses import dataclass
import numpy as np

COLOR_BLUE, COLOR_YELLOW, COLOR_ORANGE, COLOR_ORANGE_LARGE = 0, 1, 2, 3
N_CONE_CLASSES = 4

# DD.1.3.2: small cones 228 x 228 x 325 mm, large cones 285 x 285 x 505 mm.
SMALL_CONE_BASE = 0.228
LARGE_CONE_BASE = 0.285


@dataclass
class EventRules:
    """Scoring and procedure for one dynamic event (DD.4)."""

    name: str
    cone_penalty_s: float             # per cone down or out (DOO)
    off_course_penalty_s: float       # per off course (OC); None means DNF
    staging_distance_m: float         # distance from the start line at staging
    staging_reference: str            # "foremost" point or "front_wheels"
    stop_distance_m: float            # must stop within this past the finish
    time_budget_s: float              # episode truncation, not a rule
    # (fast_s, slow_s) per timed segment: times that earn the full and zero
    # timed-segment bonus in the environment. Reward shaping, not rules.
    segment_refs: tuple = ()


class CenterlineTrack:
    """Base class. Subclasses build geometry, then call `_set_centerline`,
    `_set_timing` and add cones with the helpers below."""

    # Tracker search window. Must stay shorter than the gap (in arc length)
    # between two separate passes over the same ground.
    window_back_m = 7.0
    window_fwd_m = 28.0

    # ---------------- construction helpers ----------------
    def _set_centerline(self, cx, cy):
        self.cx = np.asarray(cx, dtype=float)
        self.cy = np.asarray(cy, dtype=float)
        seg = np.hypot(np.diff(self.cx), np.diff(self.cy))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.ds = float(np.mean(seg))
        self.total_length = float(self.s[-1])
        dx = np.gradient(self.cx, edge_order=1)
        dy = np.gradient(self.cy, edge_order=1)
        self.heading = np.arctan2(dy, dx)
        h = np.unwrap(self.heading)
        self.curvature = np.gradient(h, self.s, edge_order=1)
        self._win_back = max(1, int(round(self.window_back_m / self.ds)))
        self._win_fwd = max(1, int(round(self.window_fwd_m / self.ds)))
        self._cones = []

    def _set_timing(self, crossing_idx, timed_segments, curriculum_ranges=()):
        self.crossing_idx = np.asarray(crossing_idx, dtype=int)
        self.timed_segments = list(timed_segments)
        self.curriculum_ranges = list(curriculum_ranges)
        self.crossing_turns = np.zeros(len(self.crossing_idx))

    def _finish_approach(self, v_lo, v_hi, before_m=25.0, band_m=5.0):
        """Curriculum range just before the finish line, to practise stopping."""
        f = self.finish_idx
        lo = self.idx_at_s(self.s[f] - before_m)
        hi = max(lo + 1, self.idx_at_s(self.s[f] - band_m))
        return (lo, hi, v_lo, v_hi)

    def _normal(self, i):
        h = self.heading[i]
        return np.array([-np.sin(h), np.cos(h)])

    def _add_cone(self, x, y, color):
        base = LARGE_CONE_BASE if color == COLOR_ORANGE_LARGE else SMALL_CONE_BASE
        self._cones.append((x, y, color, base / 2.0))

    def _add_boundary_cones(self, i0, i1, color, side, angle_step=0.5,
                            min_spacing=1.5, max_spacing=5.0, width=None):
        """Cones along one lane edge from index i0 to i1 (inclusive).

        side=+1 is the left edge, -1 the right. Spacing shrinks in corners to
        about `angle_step` radians of boundary arc, between the two limits.
        """
        w = self.track_width if width is None else width
        i1 = min(i1, len(self.cx) - 1)
        acc, prev = None, None
        for i in range(i0, i1 + 1):
            p = np.array([self.cx[i], self.cy[i]]) + side * (w / 2.0) * self._normal(i)
            k = self.curvature[i]
            if abs(k) < 1e-6:
                spacing = max_spacing
            else:
                r_b = abs(1.0 / k - side * w / 2.0)
                spacing = float(np.clip(angle_step * r_b, min_spacing, max_spacing))
            if prev is None:
                self._add_cone(p[0], p[1], color)
                acc = 0.0
            else:
                acc += float(np.hypot(*(p - prev)))
                if acc >= spacing:
                    self._add_cone(p[0], p[1], color)
                    acc = 0.0
            prev = p

    def _add_line_cones(self, i, offset=0.5):
        """Large orange cones just before and after a timing line (DD.1.1.d),
        placed so their inner edge lines up with the small border cones."""
        t = np.array([np.cos(self.heading[i]), np.sin(self.heading[i])])
        n = self._normal(i)
        c = np.array([self.cx[i], self.cy[i]])
        lat = self.track_width / 2.0 + (LARGE_CONE_BASE - SMALL_CONE_BASE) / 2.0
        for side in (1.0, -1.0):
            for along in (-offset, offset):
                p = c + along * t + side * lat * n
                self._add_cone(p[0], p[1], COLOR_ORANGE_LARGE)

    def _add_end_wall(self, i, spacing=1.0):
        """A row of small orange cones across the lane closing a stop area,
        at most `spacing` apart (the 1 m max marking in the DD.4 diagrams)."""
        n = self._normal(i)
        c = np.array([self.cx[i], self.cy[i]])
        half = self.track_width / 2.0
        count = int(np.ceil(2 * half / spacing)) + 1
        for lat in np.linspace(-half, half, count):
            p = c + lat * n
            self._add_cone(p[0], p[1], COLOR_ORANGE)

    def _finish_cones(self):
        if self._cones:
            arr = np.array(self._cones, dtype=float)
            self.cone_xy = arr[:, :2].copy()
            self.cone_color = arr[:, 2].astype(int)
            self.cone_radius = arr[:, 3].copy()
        else:
            self.cone_xy = np.zeros((0, 2))
            self.cone_color = np.zeros(0, dtype=int)
            self.cone_radius = np.zeros(0)
        del self._cones

    # ---------------- queries ----------------
    @property
    def start_idx(self):
        return int(self.crossing_idx[0])

    @property
    def finish_idx(self):
        return int(self.crossing_idx[-1])

    def idx_at_s(self, s_value):
        return int(np.clip(np.searchsorted(self.s, s_value), 0, len(self.s) - 1))

    def legal_cte(self, vehicle_half_width):
        """Largest centerline offset of the CG that keeps the car clear of the
        small border cones on both sides."""
        return self.track_width / 2.0 - SMALL_CONE_BASE / 2.0 - vehicle_half_width

    def staging_idx(self, front_axle_to_foremost, lf, wheel_radius):
        """Centerline index of the CG at staging, per the event's rule."""
        r = self.rules
        if r.staging_reference == "front_wheels":
            front = lf + wheel_radius
        else:
            front = lf + front_axle_to_foremost
        return self.idx_at_s(self.s[self.start_idx] - r.staging_distance_m - front)

    def lateral_offset(self, x, y, idx):
        """Signed offset of points from the centerline tangent at idx (+ left)."""
        h = self.heading[idx]
        return -np.sin(h) * (x - self.cx[idx]) + np.cos(h) * (y - self.cy[idx])

    def nearest_windowed(self, x, y, center_idx):
        """(distance, index, s, signed cross-track error, heading).

        Searches only a window around center_idx, clipped at the array ends.
        """
        N = len(self.cx)
        lo = max(0, center_idx - self._win_back)
        hi = min(N - 1, center_idx + self._win_fwd)
        d = np.hypot(self.cx[lo:hi + 1] - x, self.cy[lo:hi + 1] - y)
        local_i = int(np.argmin(d))
        idx = lo + local_i
        th = self.heading[idx]
        cte = -np.sin(th) * (x - self.cx[idx]) + np.cos(th) * (y - self.cy[idx])
        return float(d[local_i]), idx, self.s[idx], cte, th
