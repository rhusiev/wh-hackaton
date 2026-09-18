"""Trackers: turn per-frame sightings in the map frame into people that persist.

A tracker gets the sightings of one frame with update(), along with what the
camera could see in that frame. tracks() reports every track with its status, and
ar_bridge.py only depends on those two calls, so any class with them works.

NearestTracker keeps a confidence per track as log-odds. A sighting raises it. A
weak sighting may extend a track it lands on but never starts one, so a far-off
detection the detector is unsure of keeps a known person alive without inventing
a new one. A frame where the track was in plain view, close and unoccluded, but
not detected lowers it - but only once the track has gone GRACE seconds unseen,
because a detector that finds a distant person every other frame is not evidence
that nobody is there. The whole person, feet to head, has to be in frame, and the
frame's own depth image has to measure nothing in front of them. A track goes
through three statuses:

- candidate: not sure yet. It is confirmed once it has min_hits sightings and
  enough confidence, and deleted once the misses outweigh the sightings
- confirmed: a person. Misses from within miss_range make them lost
- lost: a person who is no longer where they were last seen. They keep their last
  position, get confirmed again when seen near it, and are deleted after
  lost_timeout

A person walking in view is followed frame by frame. One who walked away unseen
shows up as a new candidate; once confirmed within walking distance of a lost
person, it takes over their id.
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
UNCONFIRM = 0.0          # a confirmed track is lost below this
REJECT = -2.0            # and a candidate deleted
LOG_ODDS_MAX = 6.0
GRACE = 2.0              # s unseen before misses count, about 8 detector frames
START_SCORE = 0.6        # a sighting weaker than this may extend a track but not start one
WALK_SPEED = 1.0         # m/s, how far an unseen person may have gone
MAX_WALK = 5.0           # m, beyond which a new person is someone else
MOVED = 0.5              # m, a sighting this far off is a step, not stereo noise


@dataclass
class Sighting:
    label: str
    xyz: np.ndarray             # box centre, map frame
    height: float
    score: float
    head: np.ndarray | None     # x, y, z and size


class Tracker(Protocol):
    def update(self, sightings: list[Sighting], now: float,
               visible: Callable[[np.ndarray, float], bool]) -> None:
        """visible(xyz, max_range) says whether this frame should have detected something at xyz."""
        ...

    def tracks(self, now: float) -> list[dict]:
        """id, label, status, x, y, z, h, score, confidence, age, hits and head per track."""
        ...


STATUSES = ("confirmed", "lost", "candidate")


class Track:
    __slots__ = ("id", "label", "xyz", "height", "head", "score", "hits", "last_seen", "belief",
                 "status")

    def __init__(self, track_id: int, sighting: Sighting, now: float) -> None:
        self.id, self.label = track_id, sighting.label
        self.xyz, self.height, self.head = sighting.xyz, sighting.height, sighting.head
        self.score, self.hits, self.last_seen = sighting.score, 1, now
        self.belief, self.status = HIT, "candidate"

    def update(self, sighting: Sighting, now: float) -> None:
        self.hits += 1
        self.belief = min(self.belief + HIT, LOG_ODDS_MAX)
        # Running mean for a standing person, but a step is followed at once.
        moved = np.linalg.norm(sighting.xyz[:2] - self.xyz[:2]) > MOVED
        weight = 2 if moved else min(self.hits, 20)
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
        self.status = min(self.status, other.status, key=STATUSES.index)
        self.last_seen = max(self.last_seen, other.last_seen)

    def payload(self, now: float) -> dict:
        x, y, z = (round(float(c), 3) for c in self.xyz)
        target = {"id": self.id, "label": self.label, "status": self.status,
                  "x": x, "y": y, "z": z,
                  "h": round(self.height, 3), "score": round(self.score, 3),
                  "confidence": round(1 / (1 + math.exp(-self.belief)), 3),
                  "age": round(now - self.last_seen, 2), "hits": self.hits}
        if self.head is not None:
            target["head"] = dict(zip(("x", "y", "z", "size"),
                                      (round(float(c), 3) for c in self.head)))
        return target


class NearestTracker:
    """Sightings join the nearest track of their label within merge_radius, or start one."""

    def __init__(self, merge_radius: float, min_hits: int, max_top: float, miss_range: float,
                 lost_timeout: float) -> None:
        self.merge_radius, self.min_hits, self.max_top = merge_radius, min_hits, max_top
        self.miss_range, self.lost_timeout = miss_range, lost_timeout
        self._tracks: list[Track] = []
        self.next_id = 0

    def update(self, sightings: list[Sighting], now: float,
               visible: Callable[[np.ndarray, float], bool]) -> None:
        seen = self.associate([s for s in sightings if s.xyz[2] + s.height / 2 <= self.max_top], now)
        for track in self._tracks:
            if (track not in seen and now - track.last_seen >= GRACE
                    and all(visible(track.xyz + [0, 0, dz], self.miss_range)
                            for dz in (-track.height / 2, track.height / 2))):
                track.belief = max(track.belief - MISS, REJECT)
        for track in seen:
            self.merge_into(track)
        for track in list(self._tracks):
            if track.status == "candidate" and track.belief >= CONFIRM and track.hits >= self.min_hits:
                self.confirm(track, now)
            elif track.status == "lost" and track.belief >= CONFIRM:
                track.status = "confirmed"
            elif track.status == "confirmed" and track.belief < UNCONFIRM:
                track.status = "lost"
        self._tracks = [t for t in self._tracks if self.alive(t, now)]

    def confirm(self, track: Track, now: float) -> None:
        """A new person near where a lost one could have walked to is taken to be them."""
        track.status = "confirmed"
        lost = [(float(np.linalg.norm(t.xyz[:2] - track.xyz[:2])), t) for t in self._tracks
                if t.status == "lost" and t.label == track.label]
        distance, match = min(lost, key=lambda pair: pair[0], default=(math.inf, None))
        if match is not None and distance < min(self.merge_radius + WALK_SPEED * (now - match.last_seen),
                                                MAX_WALK):
            track.id, track.hits = match.id, track.hits + match.hits
            self._tracks.remove(match)

    def alive(self, track: Track, now: float) -> bool:
        match track.status:
            case "candidate":
                return track.belief > REJECT
            case "lost":
                return now - track.last_seen < self.lost_timeout
        return True

    def associate(self, sightings: list[Sighting], now: float) -> set[Track]:
        # Closest pairs first, so two people seen in one frame cannot end up on one track.
        pairs = sorted(
            (float(np.linalg.norm(t.xyz[:2] - s.xyz[:2])), i, j)
            for i, s in enumerate(sightings) for j, t in enumerate(self._tracks)
            if t.label == s.label)
        taken: dict[int, Track] = {}
        seen: set[Track] = set()
        for distance, i, j in pairs:
            track = self._tracks[j]
            if distance < self.merge_radius and i not in taken and track not in seen:
                taken[i] = track
                seen.add(track)
        for i, sighting in enumerate(sightings):
            if i in taken:
                taken[i].update(sighting, now)
            elif sighting.score >= START_SCORE:
                track = Track(self.next_id, sighting, now)
                self._tracks.append(track)
                self.next_id += 1
                seen.add(track)
        return seen

    def merge_into(self, track: Track) -> None:
        """Two tracks started before either position settled can end up on one person."""
        if track not in self._tracks:
            return
        for other in [t for t in self._tracks if t is not track and t.label == track.label
                      and np.linalg.norm(t.xyz[:2] - track.xyz[:2]) < self.merge_radius]:
            track.merge(other)
            self._tracks.remove(other)

    def tracks(self, now: float) -> list[dict]:
        return [t.payload(now) for t in self._tracks]
