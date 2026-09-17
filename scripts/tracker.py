"""Trackers: turn per-frame sightings in the map frame into people that persist.

A tracker gets the sightings of one frame with update() and reports the people
worth showing with targets(). ar_bridge.py only depends on those two calls, so
any class with them works.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class Sighting:
    label: str
    xyz: np.ndarray             # box centre, map frame
    height: float
    score: float
    head: np.ndarray | None     # x, y, z and size


class Tracker(Protocol):
    def update(self, sightings: list[Sighting], now: float) -> None: ...

    def targets(self, now: float) -> list[dict]:
        """The AR payload's "targets": id, label, x, y, z, h, score, age, hits and optional head."""
        ...


class Track:
    __slots__ = ("id", "label", "xyz", "height", "head", "score", "hits", "last_seen")

    def __init__(self, track_id: int, sighting: Sighting, now: float) -> None:
        self.id, self.label = track_id, sighting.label
        self.xyz, self.height, self.head = sighting.xyz, sighting.height, sighting.head
        self.score, self.hits, self.last_seen = sighting.score, 1, now

    def update(self, sighting: Sighting, now: float) -> None:
        # Running mean: repeated looks at the same person cancel the stereo noise.
        self.hits += 1
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
        self.last_seen = max(self.last_seen, other.last_seen)

    def payload(self, now: float) -> dict:
        x, y, z = (round(float(c), 3) for c in self.xyz)
        target = {"id": self.id, "label": self.label, "x": x, "y": y, "z": z,
                  "h": round(self.height, 3), "score": round(self.score, 3),
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

    def update(self, sightings: list[Sighting], now: float) -> None:
        for sighting in sightings:
            if sighting.xyz[2] + sighting.height / 2 <= self.max_top:
                self.absorb(sighting, now)

    def absorb(self, sighting: Sighting, now: float) -> None:
        # The nearest track, not the first in range: two people 1.5 m apart must not share one.
        near = [(float(np.linalg.norm(t.xyz[:2] - sighting.xyz[:2])), t) for t in self.tracks
                if t.label == sighting.label]
        distance, track = min(near, key=lambda pair: pair[0], default=(math.inf, None))
        if distance >= self.merge_radius:
            self.tracks.append(Track(self.next_id, sighting, now))
            self.next_id += 1
            return
        track.update(sighting, now)
        # Two tracks started before either position settled can end up on one person.
        for other in [t for _, t in near if t is not track
                      and np.linalg.norm(t.xyz[:2] - track.xyz[:2]) < self.merge_radius]:
            track.merge(other)
            self.tracks.remove(other)

    def targets(self, now: float) -> list[dict]:
        if self.timeout > 0:
            self.tracks = [t for t in self.tracks if now - t.last_seen < self.timeout]
        return [t.payload(now) for t in self.tracks if t.hits >= self.min_hits]
