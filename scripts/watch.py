"""Explore, then keep every person found in view from a few stations.

After FrontierExplorer has mapped the place, the confirmed people are split
between as few stations as possible. A station is a reachable spot and a heading
from which some people are within WATCH_RANGE, inside the camera's field of view
and in clear line of sight on the map. The drone flies the stations in a loop and
hovers at each, so the tracker keeps seeing everyone:

- a person walking in view is followed by the tracker
- a person a station should see but did not is looked at from up close, which is
  what makes the tracker mark them lost if they are gone
- a lost person is looked for where they were last seen, from up to three sides,
  then by turning around there
- a new candidate is inspected as during exploration
- whenever the people or their positions change, the stations are planned again
- a station it could not reach is left out of the next plans

stations() is the planner on its own; Watch flies it.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

import frontier
from coverage import HALF_FOV
from frontier import CLEARANCE, FrontierExplorer, clear_cells, distances, passable_cells
from occupancy import Grid

if TYPE_CHECKING:
    from flight import Flight, Person

WATCH_RANGE = 7.0        # inside the tracker's 8 m miss range, so a person leaving is noticed
MIN_RANGE = 1.5
SPOT_STEP = 1.0          # m between the spots considered
MOVED = 1.0              # m a person may shift before the stations are planned again
LOOKS = 3                # sides a lost person is looked for from
NEAR = 2.0               # m from where they were, within which a confirmed person is them


@dataclass
class Station:
    position: tuple[float, float]
    heading: float
    people: list[int]


def stations(grid: Grid, start: tuple[float, float], people: list[tuple[int, float, float]],
             clearance: float, avoid: list[tuple[float, float]]) -> tuple[list[Station], list[int]]:
    """Few stations that together see every person, in flying order, and who no station sees."""
    start_cell = grid.cell(*start)
    steps, _ = distances(passable_cells(grid, start_cell, clearance), start_cell)
    # Reachable and clear on its own: a cell only passable through the escape out of the
    # drone's current tight spot is not there to be flown to once the drone has moved on.
    reachable = (steps >= 0) & clear_cells(grid, clearance)
    stride = max(1, round(SPOT_STEP / grid.resolution))
    spots = [grid.point(r, c) for r, c in zip(*np.nonzero(reachable)) if r % stride == c % stride == 0]
    spots = [spot for spot in spots if all(math.dist(spot, a) > SPOT_STEP for a in avoid)]

    # Every spot and heading, with who it sees: a heading centred on each visible person.
    options: list[Station] = []
    for spot in spots:
        seen = [(pid, math.atan2(y - spot[1], x - spot[0])) for pid, x, y in people
                if MIN_RANGE <= math.dist(spot, (x, y)) <= WATCH_RANGE and grid.sees(spot, (x, y))]
        for _, heading in seen:
            options.append(Station(spot, heading, [
                pid for pid, bearing in seen if abs(math.remainder(bearing - heading, math.tau)) <= HALF_FOV]))

    # Greedy set cover, the nearer station first on a tie.
    left = {pid for pid, _, _ in people}
    chosen: list[Station] = []
    here = start
    while left and options:
        best = max(options, key=lambda o: (len(left.intersection(o.people)), -math.dist(here, o.position)))
        if not left.intersection(best.people):
            break
        chosen.append(best)
        left -= set(best.people)
        here = best.position

    # Nearest neighbour tour from the start.
    tour, here = [], start
    while chosen:
        nearest = min(chosen, key=lambda o: math.dist(here, o.position))
        chosen.remove(nearest)
        tour.append(nearest)
        here = nearest.position
    return tour, sorted(left)


class Watch:
    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        FrontierExplorer.add_arguments(parser)
        parser.add_argument("--watch-time", type=float, default=600.0,
                            help="how long to keep watching after exploring, s")
        parser.add_argument("--dwell", type=float, default=5.0,
                            help="hover at each station, s")

    def __init__(self, args: argparse.Namespace) -> None:
        self.explorer = FrontierExplorer(args)
        self.watch_time, self.dwell = args.watch_time, args.dwell
        self.searched: set[int] = set()
        self.unreachable: list[tuple[float, float]] = []

    def run(self, flight: Flight) -> None:
        self.explorer.run(flight)
        log = flight.get_logger()
        deadline = flight.get_clock().now().nanoseconds + int(self.watch_time * 1e9)
        while flight.get_clock().now().nanoseconds < deadline:
            self.explorer.look(flight)
            watched = self.confirmed(flight)
            self.searched -= {p.id for p in watched}  # found again, so lost again is new
            if not watched:
                log.info("nobody to watch")
                break
            tour, unseen = stations(flight.grid(), flight.here(),
                                    [(p.id, p.x, p.y) for p in watched], CLEARANCE, self.unreachable)
            log.info(f"watching {len(watched)} people from {len(tour)} station(s)"
                     + (f", no view of {unseen}" if unseen else ""))
            for station in tour:
                if not self.visit(flight, station):
                    log.warn(f"could not reach station at ({station.position[0]:.1f}, "
                             f"{station.position[1]:.1f})")
                    self.unreachable.append(station.position)
                    break
                for person in flight.people():
                    if person.id in station.people and person.status == "confirmed" and person.age > self.dwell:
                        log.info(f"person {person.id} not seen from the station, looking closer")
                        frontier.look_at(flight, (person.x, person.y), self.dwell)
                self.explorer.inspect(flight)
                for person in [p for p in flight.people()
                               if p.status == "lost" and p.id not in self.searched]:
                    self.search(flight, person)
                if self.changed(watched, self.confirmed(flight)):
                    break
        log.info("watch done, holding")

    @staticmethod
    def confirmed(flight: Flight) -> list[Person]:
        return [p for p in flight.people() if p.status == "confirmed"]

    @staticmethod
    def changed(before: list[Person], now: list[Person]) -> bool:
        positions = {p.id: (p.x, p.y) for p in now}
        return (positions.keys() != {p.id for p in before}
                or any(math.dist(positions[p.id], (p.x, p.y)) > MOVED for p in before))

    def visit(self, flight: Flight, station: Station) -> bool:
        reached = frontier.travel(flight, lambda grid: frontier.plan_path(
            grid, flight.here(), station.position, CLEARANCE, station.heading))
        if reached:
            flight.wait(lambda: False, self.dwell)
        return reached

    def search(self, flight: Flight, person: Person) -> None:
        """Look where a lost person was from several sides, then turn around there."""
        self.searched.add(person.id)
        log = flight.get_logger()
        log.info(f"person {person.id} lost at ({person.x:.1f}, {person.y:.1f}) {person.age:.0f} s ago")

        def found() -> bool:
            # Their own id if the tracker kept it, otherwise whoever is confirmed there now:
            # a person whose track died and started over comes back under a new id.
            return any(p.status == "confirmed" and p.age < self.dwell
                       and (p.id == person.id or math.dist((p.x, p.y), (person.x, person.y)) < NEAR)
                       for p in flight.people())

        # One side may be the one a hedge or a rack hides them from, so try others.
        tried: list[float] = []
        for _ in range(LOOKS):
            bearing = frontier.look_at(flight, (person.x, person.y), self.dwell, tried)
            if bearing is None:
                break
            tried.append(bearing)
            if found():
                log.info(f"person {person.id} found again from {len(tried)} side(s)")
                return
        for _ in range(4):
            flight.turn(math.remainder(flight.heading() + math.pi / 2, math.tau))
            if flight.wait(found, self.dwell / 2):
                log.info(f"person {person.id} found again")
                return
        log.warn(f"person {person.id} not found near their last position")
