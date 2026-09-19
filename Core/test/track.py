"""
Track entry point. One builder per Driverless dynamic event:

    SkidpadTrack        track_skidpad.py   (DD.4.3)
    AccelerationTrack   track_accel.py     (DD.4.2)
    AutocrossTrack      track_autocross.py (DD.4.4, DD.4.5), randomly generated

    track = make_track("autocross", seed=3)

`Figure8Track` is kept as the old name of SkidpadTrack.
"""
from track_base import (CenterlineTrack, EventRules, COLOR_BLUE, COLOR_YELLOW,
                        COLOR_ORANGE, COLOR_ORANGE_LARGE)
from track_skidpad import SkidpadTrack, POINTS_PER_LOOP, FSAE_LAP_SEQUENCE
from track_accel import AccelerationTrack
from track_autocross import AutocrossTrack, GenerationError

Figure8Track = SkidpadTrack

EVENTS = {
    "skidpad": SkidpadTrack,
    "acceleration": AccelerationTrack,
    "accel": AccelerationTrack,
    "autocross": AutocrossTrack,
}


def make_track(event, **kwargs):
    try:
        cls = EVENTS[event.lower()]
    except KeyError:
        raise ValueError(f"unknown event {event!r}; choose from {sorted(set(EVENTS))}")
    return cls(**kwargs)
