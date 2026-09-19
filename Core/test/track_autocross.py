"""
Autocross track generator (DD.4.4 layout, DD.4.5 event).

Builds a random closed course within the DD.4.4.1 limits:
    a. straights no longer than 80 m
    b. chicanes, multiple turns, decreasing radius turns, hairpins
    c. track width at least 3 m
    d. minimum turning diameter 9 m (taken as a 4.5 m centerline radius)
    e. lap length 200 - 500 m

The car is staged with its front wheels 6 m behind the start line (DD.4.4.3),
runs one lap (DD.4.5.1), and must stop within 30 m on the track after the
finish (DD.4.4.4). 2 s per cone down or out, cones after the finish included,
and 10 s per off course (DD.4.5.3).

How a course is built: a random sequence of features (straights and arcs) is
drawn, closing corners are added so the heading turns exactly one full circle,
and the straight lengths are then solved so the path ends where it began. A
candidate is rejected if it breaks a limit above or if two separate parts of the
course come closer than `min_separation` (centerline to centerline), which also
rules out crossings. The start line sits on the first straight.

Not from the rules, and worth tuning to the courses you expect: the feature mix
and sizes, `min_separation`, and the border cone spacing (tighter in corners).

Usage:

    track = AutocrossTrack(seed=3)
    track.lap_length, track.segments     # the feature list it was built from
"""
from dataclasses import dataclass, field
import numpy as np
from scipy.spatial import cKDTree

from track_base import CenterlineTrack, EventRules, COLOR_BLUE, COLOR_YELLOW


def autocross_rules(lap_length):
    return EventRules(
        name="autocross",
        cone_penalty_s=2.0,              # DD.4.5.3.a
        off_course_penalty_s=10.0,       # DD.4.5.3.b
        staging_distance_m=6.0,          # DD.4.4.3, front wheels
        staging_reference="front_wheels",
        stop_distance_m=30.0,            # DD.4.4.4.a
        time_budget_s=max(60.0, lap_length / 4.0 + 20.0),
        segment_refs=((lap_length / 15.0, lap_length / 6.0),),
    )


def _arc_step(p, h, R, dth):
    """End point and heading after an arc of radius R turning dth (+ left)."""
    sgn = np.sign(dth)
    d = sgn * R * np.array([np.sin(h + dth) - np.sin(h), np.cos(h) - np.cos(h + dth)])
    return p + d, h + dth


class GenerationError(RuntimeError):
    pass


@dataclass
class AutocrossTrack(CenterlineTrack):
    seed: int = None
    track_width: float = 3.0              # DD.4.4.1.c (minimum)
    min_radius: float = 4.5               # DD.4.4.1.d
    max_straight: float = 80.0            # DD.4.4.1.a
    lap_length_range: tuple = (200.0, 500.0)   # DD.4.4.1.e
    target_length_range: tuple = (250.0, 450.0)
    min_separation: float = 5.0           # assumed
    start_line_offset: float = 10.0       # start line position on the first straight
    first_straight_min: float = 25.0
    ds_target: float = 0.1
    max_attempts: int = 400
    rules: EventRules = None

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        last_reason = ""
        for _ in range(self.max_attempts):
            segs = self._draw_segments(rng)
            segs, reason = self._close(segs)
            if segs is None:
                last_reason = reason
                continue
            loop, reason = self._render(segs)
            if loop is None:
                last_reason = reason
                continue
            self.segments = segs
            self._build(loop)
            return
        raise GenerationError(f"no valid course in {self.max_attempts} attempts "
                              f"(last rejection: {last_reason})")

    # ---------------- feature drawing ----------------
    def _draw_segments(self, rng):
        deg = np.radians
        Rmin = self.min_radius
        target = rng.uniform(*self.target_length_range)
        segs = [["S", rng.uniform(30.0, 60.0)]]
        length = segs[0][1]

        def seg_len(s):
            return s[1] if s[0] == "S" else s[1] * abs(s[2])

        while length < 0.8 * target:
            kind = rng.choice(["corner", "hairpin", "chicane", "decreasing", "esses"],
                              p=[0.35, 0.15, 0.18, 0.14, 0.18])
            sgn = rng.choice([-1.0, 1.0])
            if kind == "corner":
                feat = [["A", rng.uniform(6.0, 25.0), sgn * deg(rng.uniform(40, 120))]]
            elif kind == "hairpin":
                feat = [["A", rng.uniform(Rmin, 7.0), sgn * deg(rng.uniform(160, 195))]]
            elif kind == "chicane":
                R, a = rng.uniform(8.0, 16.0), deg(rng.uniform(25, 50))
                feat = [["A", R, sgn * a], ["S", rng.uniform(2.0, 6.0)],
                        ["A", R, -2 * sgn * a], ["S", rng.uniform(2.0, 6.0)],
                        ["A", R, sgn * a]]
            elif kind == "decreasing":
                R1 = rng.uniform(14.0, 22.0)
                R2 = max(Rmin, R1 * rng.uniform(0.55, 0.75))
                R3 = max(Rmin, R2 * rng.uniform(0.55, 0.75))
                feat = [["A", R, sgn * deg(rng.uniform(25, 50))] for R in (R1, R2, R3)]
            else:  # esses: several alternating turns
                feat = []
                for j in range(int(rng.integers(2, 5))):
                    feat.append(["A", rng.uniform(8.0, 15.0),
                                 (sgn if j % 2 == 0 else -sgn) * deg(rng.uniform(40, 80))])
            if segs[-1][0] == "A" and rng.random() < 0.7:
                segs.append(["S", rng.uniform(8.0, 45.0)])
            segs.extend(feat)
            length = sum(seg_len(s) for s in segs)

        # Closing corners: bring the total heading change to exactly +-2 pi.
        total = sum(s[2] for s in segs if s[0] == "A")
        goal = 2 * np.pi if total >= 0 else -2 * np.pi
        rem = goal - total
        m = max(1, int(np.ceil(abs(rem) / deg(130))))
        for _ in range(m):
            pos = int(rng.integers(1, len(segs) + 1))
            segs[pos:pos] = [["S", rng.uniform(8.0, 30.0)],
                             ["A", rng.uniform(7.0, 20.0), rem / m]]
        return segs

    # ---------------- closure ----------------
    def _close(self, segs):
        # Merge touching straights, including the end of the lap onto the start.
        merged = []
        for s in segs:
            if merged and s[0] == "S" and merged[-1][0] == "S":
                merged[-1][1] += s[1]
            else:
                merged.append(list(s))
        if len(merged) > 1 and merged[-1][0] == "S":
            merged[0][1] += merged.pop()[1]
        segs = merged

        straight_ids = [i for i, s in enumerate(segs) if s[0] == "S"]
        if len(straight_ids) < 3:
            return None, "too few straights to close the loop"
        lo = np.array([self.first_straight_min if i == 0 else 4.0 for i in straight_ids])
        hi = np.full(len(straight_ids), self.max_straight)
        L = np.clip(np.array([segs[i][1] for i in straight_ids]), lo, hi)

        # Direction of each straight and the path end point for lengths L.
        dirs, h = [], 0.0
        for s in segs:
            if s[0] == "S":
                dirs.append((np.cos(h), np.sin(h)))
            else:
                h += s[2]
        D = np.array(dirs).T                                     # 2 x m

        def end_point(lengths):
            p, h, k = np.zeros(2), 0.0, 0
            for s in segs:
                if s[0] == "S":
                    p = p + lengths[k] * np.array([np.cos(h), np.sin(h)])
                    k += 1
                else:
                    p, h = _arc_step(p, h, s[1], s[2])
            return p

        free = np.ones(len(L), dtype=bool)
        for _ in range(len(L) + 1):
            gap = end_point(L)
            if np.hypot(*gap) < 1e-6:
                break
            Df = D[:, free]
            W = np.diag(L[free])
            A = Df @ W @ Df.T
            if not free.any() or np.linalg.cond(A) > 1e8:
                return None, "straights cannot close the gap"
            dL = W @ Df.T @ np.linalg.solve(A, -gap)
            newL = L.copy()
            newL[free] += dL
            viol = (newL < lo - 1e-9) | (newL > hi + 1e-9)
            if not viol.any():
                L = newL
                continue
            # Pin the violating straights at their bounds and solve again.
            L = np.clip(newL, lo, hi)
            free &= ~viol
        else:
            return None, "closure did not converge"
        if np.hypot(*end_point(L)) > 1e-6:
            return None, "closure left a gap"
        for k, i in enumerate(straight_ids):
            segs[i][1] = float(L[k])
        return segs, ""

    # ---------------- geometry checks ----------------
    def _render(self, segs, step=0.05):
        pts, p, h = [np.zeros(2)], np.zeros(2), 0.0
        for s in segs:
            if s[0] == "S":
                n = max(1, int(np.ceil(s[1] / step)))
                t = np.arange(1, n + 1) / n * s[1]
                pts.extend(p + np.outer(t, [np.cos(h), np.sin(h)]))
                p = pts[-1]
            else:
                R, dth = s[1], s[2]
                n = max(1, int(np.ceil(R * abs(dth) / step)))
                for a in np.arange(1, n + 1) / n * dth:
                    q, _ = _arc_step(p, h, R, a)
                    pts.append(q)
                p, h = _arc_step(p, h, R, dth)
        P = np.array(pts)[:-1]                     # last point duplicates the first
        seg = np.hypot(*np.diff(np.vstack([P, P[:1]]), axis=0).T)
        lap = float(seg.sum())
        lo, hi = self.lap_length_range
        if not lo <= lap <= hi:
            return None, f"lap length {lap:.0f} m outside {lo:.0f}-{hi:.0f} m"

        # Uniform resampling of the closed loop.
        n = int(round(lap / self.ds_target))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        closed = np.vstack([P, P[:1]])
        t = np.arange(n) * lap / n
        loop = np.column_stack([np.interp(t, cum, closed[:, 0]),
                                np.interp(t, cum, closed[:, 1])])

        # Separation between distinct parts of the course.
        ds = lap / n
        sep = self.min_separation
        pairs = cKDTree(loop).query_pairs(sep, output_type="ndarray")
        if len(pairs):
            gap = np.abs(pairs[:, 0] - pairs[:, 1])
            arc = np.minimum(gap, n - gap) * ds
            if np.any(arc > 0.5 * np.pi * sep):
                return None, "course comes too close to itself"
        return loop, ""

    # ---------------- final track ----------------
    def _build(self, loop):
        n = len(loop)
        self.lap_length = float(np.hypot(*np.diff(np.vstack([loop, loop[:1]]), axis=0).T).sum())
        if self.rules is None:
            self.rules = autocross_rules(self.lap_length)
        ds = self.lap_length / n
        pre = self.start_line_offset          # staged car sits on the first straight
        post = self.rules.stop_distance_m + 10.0
        i_start = int(round(self.start_line_offset / ds))
        pre_n = int(round(pre / ds))
        post_n = int(round(post / ds))
        order = (i_start - pre_n + np.arange(pre_n + n + post_n + 1)) % n
        self._set_centerline(loop[order, 0], loop[order, 1])

        start, finish = pre_n, pre_n + n
        band = int(round(5.0 / ds))
        self._set_timing([start, finish], [("lap", 0, 1)],
                         [(start + band, finish - band, 3.0, 8.0)])
        self.curriculum_ranges.append(self._finish_approach(4.0, 9.0))
        for side, color in ((1, COLOR_BLUE), (-1, COLOR_YELLOW)):
            self._add_boundary_cones(start, finish - 1, color, side)
        self._add_line_cones(start)
        self._finish_cones()
