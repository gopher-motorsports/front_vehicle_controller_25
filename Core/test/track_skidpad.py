"""
Skidpad track (DD.4.3, layout per FSAE Rules D.10.1).

Two circles tangent at a shared start/finish gate. The car is staged in the
entry lane with its foremost part 15 m before the gate, drives the right circle
twice, then the left circle twice, and must stop within 25 m in the exit lane.
The second lap on each circle is timed; the result is their average plus
0.125 s per cone down or out, entry and exit lane cones included. Off course,
a wrong lap count or wrong order is a DNF.

Dimensions: 15.25 m inner circle diameter and a 3 m lane, so an 18.25 m
centerline diameter. These come from the D.10.1 layout as recalled, not from
the supplement, so check them against the Rules. 17 pylons around the inside of
each inner circle is DD.4.3.1.b. The outer pylon count and the lane cone
spacing are assumptions.

Colours follow the DD.4.3 diagram: left circle inner blue, outer yellow; right
circle inner yellow, outer blue. Outer cones that would fall where the two
circles overlap are left out.

Usage:

    track = SkidpadTrack()
    track.circle_diameter, track.track_width
    track.crossing_idx          # gate crossings, entry -> exit
    track.timed_segments        # [("right", 1, 2), ("left", 3, 4)]
"""
from dataclasses import dataclass, field
import numpy as np

from track_base import (CenterlineTrack, EventRules, COLOR_BLUE, COLOR_YELLOW,
                        COLOR_ORANGE, SMALL_CONE_BASE)

POINTS_PER_LOOP = 500        # centerline samples per loop (numerical resolution)
FSAE_LAP_SEQUENCE = "RRLL"   # D.10: two laps right, then two laps left


def skidpad_rules():
    return EventRules(
        name="skidpad",
        cone_penalty_s=0.125,            # DD.4.3.5.a
        off_course_penalty_s=None,       # DD.4.3.5.b: DNF
        staging_distance_m=15.0,         # DD.4.3.2, foremost part
        staging_reference="foremost",
        stop_distance_m=25.0,            # DD.4.3.4.a
        time_budget_s=60.0,
        segment_refs=((4.5, 10.0), (4.5, 10.0)),
    )


@dataclass
class SkidpadTrack(CenterlineTrack):
    circle_diameter: float = 18.25   # centerline: 15.25 m inner + 3 m lane (verify, D.10.1)
    track_width: float = 3.0         # (verify, D.10.1)
    n_inner_pylons: int = 17         # DD.4.3.1.b
    n_outer_pylons: int = 17         # assumed
    lane_cone_spacing: float = 2.5   # entry/exit lane orange cones (assumed)
    lap_sequence: str = FSAE_LAP_SEQUENCE
    rules: EventRules = field(default_factory=skidpad_rules)

    def __post_init__(self):
        seq = self.lap_sequence.upper()
        if not seq or set(seq) - {"R", "L"}:
            raise ValueError(f"lap_sequence must be made of 'R' and 'L', got {self.lap_sequence!r}")
        self.lap_sequence = seq
        n = POINTS_PER_LOOP
        R = self.circle_diameter / 2.0
        ds = 2 * np.pi * R / n
        self.points_per_loop = n
        self.n_loops = len(seq)
        self.loop_length = 2 * np.pi * R
        self.r_inner = R - self.track_width / 2.0
        self.r_outer = R + self.track_width / 2.0

        # Entry lane up to the gate at (0, 0), heading +y. Long enough for the
        # staged car (15 m plus its front overhang) with room behind it.
        entry_len = self.rules.staging_distance_m + 6.0
        m_in = int(round(entry_len / ds))
        entry_y = -ds * np.arange(m_in, 0, -1)

        theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
        loops = {
            "L": (-R + R * np.cos(theta), R * np.sin(theta)),   # counter-clockwise
            "R": (R - R * np.cos(theta), R * np.sin(theta)),    # clockwise
        }
        # Exit lane from the gate: the stop area plus room to overshoot it.
        exit_len = self.rules.stop_distance_m + 10.0
        m_out = int(round(exit_len / ds)) + 1
        exit_y = ds * np.arange(m_out)

        cx = np.concatenate([np.zeros(m_in)] + [loops[d][0] for d in seq] + [np.zeros(m_out)])
        cy = np.concatenate([entry_y] + [loops[d][1] for d in seq] + [exit_y])
        self._set_centerline(cx, cy)

        crossings = [m_in + k * n for k in range(self.n_loops + 1)]
        segments, seen = [], set()
        for k in range(1, self.n_loops):
            if seq[k] == seq[k - 1] and seq[k] not in seen:
                seen.add(seq[k])
                segments.append(("right" if seq[k] == "R" else "left", k, k + 1))
        band = int(round(5.0 / ds))
        # Late in every loop, so each gate decision (continue on this circle,
        # switch circle, or exit) is practised equally. The loop a car spawns
        # in is not timed.
        curriculum = [(m_in + int((k + 0.75) * n), m_in + (k + 1) * n - band, 3.0, 6.0)
                      for k in range(self.n_loops)]
        self._set_timing(crossings, segments, curriculum)
        # After gate crossing k the car drives loop k (right is -1, left +1);
        # after the last crossing it goes straight into the exit lane.
        self.crossing_turns = np.array(
            [(-1.0 if d == "R" else 1.0) for d in seq] + [0.0])
        self.entry_points = m_in
        self._build_cones(R, m_in)

    def _build_cones(self, R, m_in):
        r_in, r_out = self.r_inner, self.r_outer
        th_in = np.linspace(0, 2 * np.pi, self.n_inner_pylons, endpoint=False)
        th_out = np.linspace(0, 2 * np.pi, self.n_outer_pylons, endpoint=False)
        in_xy = np.column_stack([-R + r_in * np.cos(th_in), r_in * np.sin(th_in)])
        out_xy = np.column_stack([-R + r_out * np.cos(th_out), r_out * np.sin(th_out)])
        keep = np.hypot(out_xy[:, 0] - R, out_xy[:, 1]) >= r_out - 1e-6
        out_xy = out_xy[keep]

        # Left circle: inner blue, outer yellow. Right circle is the mirror
        # image with the colours swapped.
        for (x, y) in in_xy:
            self._add_cone(x, y, COLOR_BLUE)
            self._add_cone(-x, y, COLOR_YELLOW)
        for (x, y) in out_xy:
            self._add_cone(x, y, COLOR_YELLOW)
            self._add_cone(-x, y, COLOR_BLUE)

        # Entry and exit lanes: small orange cones on both edges, only where
        # they sit clear of both circles.
        half = self.track_width / 2.0
        clear = r_out + 0.5
        stop_y = self.rules.stop_distance_m
        entry_y = -self.rules.staging_distance_m
        ys = np.concatenate([
            np.arange(entry_y, 0.0, self.lane_cone_spacing),
            np.arange(stop_y, 0.0, -self.lane_cone_spacing),
        ])
        for y in ys:
            for x in (-half, half):
                if np.hypot(x - R, y) > clear and np.hypot(x + R, y) > clear:
                    self._add_cone(x, y, COLOR_ORANGE)

        self._add_line_cones(m_in)
        self._add_end_wall(self.idx_at_s(self.s[self.finish_idx] + stop_y))
        self._finish_cones()
