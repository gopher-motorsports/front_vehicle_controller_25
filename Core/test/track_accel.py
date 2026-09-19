"""
Acceleration track (DD.4.2, layout per FSAE Rules D.9.1).

A straight lane. The car is staged with its foremost part 0.3 m behind the
start line, timed from the start line to the finish line 75 m later, and must
come to a full stop within 75 m past the finish, inside the exit lane.
2 s per cone down or out (entry and exit gate cones included); off course is a
DNF.

From the DD.4.2 diagram: 75 m start to finish, 75 m stop area, 3 m minimum lane
width, stop-area cones about 5 m apart, and cones across the end of the stop
area at most 1 m apart. The border cone spacing is not given there and is an
assumption; D.9.1 in the main Rules may also set a wider lane, so check it.

Usage:

    track = AccelerationTrack()                 # or AccelerationTrack(track_width=4.9)
"""
from dataclasses import dataclass, field
import numpy as np

from track_base import (CenterlineTrack, EventRules, COLOR_BLUE, COLOR_YELLOW,
                        COLOR_ORANGE)


def acceleration_rules():
    return EventRules(
        name="acceleration",
        cone_penalty_s=2.0,              # DD.4.2.5.a
        off_course_penalty_s=None,       # DD.4.2.5.b: DNF
        staging_distance_m=0.3,          # DD.4.2.2, foremost part
        staging_reference="foremost",
        stop_distance_m=75.0,            # DD.4.2.4.a
        time_budget_s=25.0,
        segment_refs=((3.2, 7.0),),
    )


@dataclass
class AccelerationTrack(CenterlineTrack):
    run_length: float = 75.0          # start line to finish line (DD.4.2 diagram)
    track_width: float = 3.0          # "3 m min." in the diagram (verify, D.9.1)
    border_cone_spacing: float = 5.0  # assumed
    stop_cone_spacing: float = 5.0    # "~5 m" in the diagram
    ds_target: float = 0.1
    rules: EventRules = field(default_factory=acceleration_rules)

    def __post_init__(self):
        pre = self.rules.staging_distance_m + 6.0      # staged car plus room behind
        post = self.rules.stop_distance_m + 10.0       # stop area plus overrun
        total = pre + self.run_length + post
        n = int(round(total / self.ds_target)) + 1
        x = np.linspace(-pre, self.run_length + post, n)
        self._set_centerline(x, np.zeros_like(x))

        start = self.idx_at_s(pre)
        finish = self.idx_at_s(pre + self.run_length)
        # Rolling spawns along the run (practise running at speed) and just
        # before the finish (practise stopping). Neither is timed.
        run_range = (self.idx_at_s(pre + 2.0), self.idx_at_s(pre + self.run_length - 25.0),
                     4.0, 18.0)
        self._set_timing([start, finish], [("run", 0, 1)], [run_range])
        self.curriculum_ranges.append(self._finish_approach(12.0, 22.0))

        stop_end = self.idx_at_s(self.s[finish] + self.rules.stop_distance_m)
        for side, color in ((1, COLOR_BLUE), (-1, COLOR_YELLOW)):
            self._add_boundary_cones(start, finish, color, side,
                                     max_spacing=self.border_cone_spacing)
            self._add_boundary_cones(finish + int(round(self.stop_cone_spacing / self.ds)),
                                     stop_end, COLOR_ORANGE, side,
                                     max_spacing=self.stop_cone_spacing)
        self._add_line_cones(start)
        self._add_line_cones(finish)
        self._add_end_wall(stop_end)
        self._finish_cones()
