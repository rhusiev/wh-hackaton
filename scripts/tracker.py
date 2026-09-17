"""Trackers: turn per-frame sightings in the map frame into people that persist.

A tracker gets the sightings of one frame with update(), along with what the
camera could see in that frame. It reports the people worth showing with
targets() and the ones it is still unsure of with candidates(). ar_bridge.py only
depends on those three calls, so any class with them works.

NearestTracker keeps a confidence per track as log-odds. A sighting raises it. A
frame where the track was in plain view, close and unoccluded, but not detected
lowers it. A track is shown once it has min_hits sightings and enough confidence,
and dropped once the misses outweigh the sightings. So a closer look settles a
candidate either way.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np

HIT = 0.5
MISS = 0.35
CONFIRM = 2.0            # with min_hits, about 6 clean sightings
UNCONFIRM = 0.0          # a shown track is hidden again below this
REJECT = -2.0            # and deleted below this
LOG_ODDS_MAX = 6.0


@dataclass
class Sighting:
    label: str
    xyz: np.ndarray             # box centre, map frame
    height: float
    score: float
    head: np.ndarray | None     # x, y, z and size


class Tracker(Protocol):
    def update(self, sightings: list[Sighting], now: float,
               visible: Callable[[np.ndarray], bool]) -> None:
        """visible(xyz) says whether this frame should have detected something at xyz."""
        ...

    def targets(self, now: float) -> list[dict]:
        """The AR payload's "targets": id, label, x, y, z, h, score, confidence, age, hits, head."""
        ...

    def candidates(self, now: float) -> list[dict]:
        """Tracks that may be people but are not shown yet, in the same form."""
        ...


class Track:
    __slots__ = ("id", "label", "xyz", "height", "head", "score", "hits", "last_seen", "belief",
                 "confirmed")

    def __init__(self, track_id: int, sighting: Sighting, now: float) -> None:
        self.id, self.label = track_id, sighting.label
        self.xyz, self.height, self.head = sighting.xyz, sighting.height, sighting.head
        self.score, self.hits, self.last_seen = sighting.score, 1, now
        self.belief, self.confirmed = HIT, False

    def update(self, sighting: Sighting, now: float) -> None:
        # Running mean: repeated looks at the same person cancel the stereo noise.
        self.hits += 1
        self.belief = min(self.belief + HIT, LOG_ODDS_MAX)
        weight = min(self.hits, 20)
        self.xyz = self.xyz + (sighting.xyz - self.xyz) / weight
        self.height += (sighting.height - self.height) / weight
        if sighting.head is not None:
            self.head = (sighting.head if self.head is None
                         else self.head + (sighting.head - self.head) / weight)
        self.score = max(self.score, sighting.score)
        self.last_seen = now

    def merge(self, other: Track) -> None:
        """Take over a track that turned out to be the same person, weighted by sightings."""
        share = other.hits / (self.hits + other.hits)
        self.xyz = self.xyz + (other.xyz - self.xyz) * share
        self.height += (other.height - self.height) * share
        if other.head is not None:
            self.head = other.head if self.head is None else self.head + (other.head - self.head) * share
        self.score = max(self.score, other.score)
        self.hits += other.hits
        self.belief = min(max(self.belief, other.belief), LOG_ODDS_MAX)
        self.confirmed |= other.confirmed
        self.last_seen = max(self.last_seen, other.last_seen)

    def payload(self, now: float) -> dict:
        x, y, z = (round(float(c), 3) for c in self.xyz)
        target = {"id": self.id, "label": self.label, "x": x, "y": y, "z": z,
                  "h": round(self.height, 3), "score": round(self.score, 3),
                  "confidence": round(1 / (1 + math.exp(-self.belief)), 3),
                  "age": round(now - self.last_seen, 2), "hits": self.hits}
        if self.head is not None:
            target["head"] = dict(zip(("x", "y", "z", "size"),
                                      (round(float(c), 3) for c in self.head)))
        return target


class NearestTracker:
    """Each sighting joins the nearest track of its label within merge_radius, or starts one."""

    def __init__(self, merge_radius: float, min_hits: int, max_top: float, timeout: float) -> None:
        self.merge_radius, self.min_hits = merge_radius, min_hits
        self.max_top, self.timeout = max_top, timeout
        self.tracks: list[Track] = []
        self.next_id = 0

    def update(self, sightings: list[Sighting], now: float,
               visible: Callable[[np.ndarray], bool]) -> None:
        seen = {self.absorb(s, now) for s in sightings if s.xyz[2] + s.height / 2 <= self.max_top}
        for track in self.tracks:
            if track not in seen and visible(track.xyz):
                track.belief -= MISS
        for track in self.tracks:
            if track.hits >= self.min_hits and track.belief >= CONFIRM:
                track.confirmed = True
            elif track.belief < UNCONFIRM:
                track.confirmed = False
        self.tracks = [t for t in self.tracks if t.belief > REJECT]

    def absorb(self, sighting: Sighting, now: float) -> Track:
        # The nearest track, not the first in range: two people 1.5 m apart must not share one.
        near = [(float(np.linalg.norm(t.xyz[:2] - sighting.xyz[:2])), t) for t in self.tracks
                if t.label == sighting.label]
        distance, track = min(near, key=lambda pair: pair[0], default=(math.inf, None))
        if distance >= self.merge_radius:
            track = Track(self.next_id, sighting, now)
            self.tracks.append(track)
            self.next_id += 1
            return track
        track.update(sighting, now)
        # Two tracks started before either position settled can end up on one person.
        for other in [t for _, t in near if t is not track
                      and np.linalg.norm(t.xyz[:2] - track.xyz[:2]) < self.merge_radius]:
            track.merge(other)
            self.tracks.remove(other)
        return track

    def targets(self, now: float) -> list[dict]:
        if self.timeout > 0:
            self.tracks = [t for t in self.tracks if now - t.last_seen < self.timeout]
        return [t.payload(now) for t in self.tracks if t.confirmed]

    def candidates(self, now: float) -> list[dict]:
        return [t.payload(now) for t in self.tracks if not t.confirmed]
