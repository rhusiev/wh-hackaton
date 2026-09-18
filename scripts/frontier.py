"""Frontier exploration on an occupancy grid: where to fly next, and how to get there.

A frontier is known free space next to unknown space. Flying to the nearest one
and looking into the unknown grows the map until no reachable frontier is left.
plan(), plan_view() and plan_path() are the planner on its own; leg(), travel()
and look_at() fly a plan; FrontierExplorer puts them together.

A complete map is not a searched place, so the explorer does not stop there: it
then flies to whatever the camera has never had in view (coverage.py) until
there is nothing left to look at. It switches over as soon as chasing frontiers
stops paying, whether none is left or the map has simply not grown for STALE
seconds, and switches back if a look reveals new ground.

Between legs it also takes a closer look at candidates, people the tracker is not
sure of yet: it flies to a spot a few metres away with a clear view, faces them
and hovers. A real person keeps being detected and gets confirmed, a false one
stops being detected and loses confidence.
"""

from __future__ import annotations

import argparse
import math
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

from coverage import HALF_FOV, LOOK_RANGE, Coverage
from occupancy import OCCUPIED, UNKNOWN, Grid

if TYPE_CHECKING:
    from flight import Flight

# Legs are short so each plan uses the map the last leg revealed.
LEG = 3.0
CLEARANCE = 1.0          # 0.4 m arm tip, plus the 0.6 m the drone may still be from a waypoint
MIN_CELLS = 4
VISITED_RADIUS = 1.5     # a frontier still unknown after looking at it is given up

# Candidates: people the tracker is not sure of yet.
INSPECT_BELOW = 0.9      # confidence under which a candidate is worth a detour
INSPECT_RANGE = 5.0      # from 2.8 m up the feet leave the frame closer than about 4.2 m
INSPECT_HOVER = 3.0      # s of looking, ~12 detector frames
MAX_INSPECTIONS = 2
APART = 1.0              # rad between one look at a spot and the next

# Places the map knows but the camera never had in view.
MIN_LOOK = 12            # cells, about half a square metre, below which a gap is not worth a trip
LOOK_HOVER = 2.0         # s of looking at one
STALE = 60.0             # s without real growth before frontiers are not worth chasing
GROWTH = 100             # cells, 4 m2: fewer than this in STALE is noise, not exploring
GIVE_UP = 12             # legs spent on one frontier before it is written off

NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Goal:
    path: list[tuple[float, float]]  # waypoints after the start, the goal last
    look_yaw: float                  # where to face on arrival


def _disk(radius: int) -> np.ndarray:
    r = np.arange(-radius, radius + 1)
    return r[:, None] ** 2 + r[None, :] ** 2 <= radius ** 2


def distances(passable: np.ndarray, start: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Breadth-first steps from start over passable cells, and each cell's parent index."""
    rows, cols = passable.shape
    steps = np.full(passable.shape, -1, dtype=np.int32)
    parent = np.full(passable.shape, -1, dtype=np.int64)
    steps[start] = 0
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for dr, dc in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and passable[nr, nc] and steps[nr, nc] < 0:
                steps[nr, nc] = steps[r, c] + 1
                parent[nr, nc] = r * cols + c
                queue.append((nr, nc))
    return steps, parent


def _line_clear(passable: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) + 1
    rows = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    cols = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(passable[rows, cols].all())


def _free(grid: Grid) -> np.ndarray:
    return (grid.cells >= 0) & (grid.cells < OCCUPIED)


def room_around(grid: Grid) -> np.ndarray:
    """Cells to the nearest occupied cell, for every cell."""
    return ndimage.distance_transform_edt(grid.cells < OCCUPIED)


def clear_cells(grid: Grid, clearance: float, room: np.ndarray | None = None) -> np.ndarray:
    """Known free cells at least clearance away from anything occupied."""
    room = room_around(grid) if room is None else room
    return _free(grid) & (room > math.ceil(clearance / grid.resolution))


def passable_cells(grid: Grid, start_cell: tuple[int, int], clearance: float) -> np.ndarray:
    margin = math.ceil(clearance / grid.resolution)
    room = room_around(grid)
    passable = clear_cells(grid, clearance, room)
    # The drone may hover inside the clearance margin; it still has to leave from there,
    # but only through cells no closer to anything than the one it is already in.
    escape = np.zeros_like(passable)
    escape[max(start_cell[0] - margin, 0):start_cell[0] + margin + 1,
           max(start_cell[1] - margin, 0):start_cell[1] + margin + 1] = True
    passable |= escape & _free(grid) & (room >= room[start_cell])
    passable[start_cell] = True
    return passable


def _corners(grid: Grid, passable: np.ndarray, parent: np.ndarray, start_cell: tuple[int, int],
             goal_cell: tuple[int, int]) -> list[tuple[float, float]]:
    cols = passable.shape[1]
    chain = [goal_cell]
    while chain[-1] != start_cell:
        chain.append(divmod(int(parent[chain[-1]]), cols))
    chain.reverse()

    # Keep only the corners: each waypoint is the farthest cell still in line of sight.
    corners, i = [], 0
    while i < len(chain) - 1:
        j = len(chain) - 1
        while j > i + 1 and not _line_clear(passable, chain[i], chain[j]):
            j -= 1
        corners.append(chain[j])
        i = j
    return [grid.point(*cell) for cell in corners]


def _inside(grid: Grid, cell: tuple[int, int]) -> bool:
    return 0 <= cell[0] < grid.cells.shape[0] and 0 <= cell[1] < grid.cells.shape[1]


def plan(grid: Grid, start: tuple[float, float], clearance: float, min_cells: int,
         avoid: list[tuple[float, float]], avoid_radius: float) -> Goal | None:
    """Path to the nearest reachable frontier, or None when the map is explored."""
    start_cell = grid.cell(*start)
    if not _inside(grid, start_cell):
        return None
    passable = passable_cells(grid, start_cell, clearance)
    unknown = grid.cells == UNKNOWN
    rows, cols = grid.cells.shape

    frontier = passable & ndimage.binary_dilation(unknown, np.ones((3, 3), bool))
    ys, xs = np.mgrid[0:rows, 0:cols]
    for x, y in avoid:
        r, c = grid.cell(x, y)
        frontier &= (ys - r) ** 2 + (xs - c) ** 2 > (avoid_radius / grid.resolution) ** 2

    steps, parent = distances(passable, start_cell)
    labels, count = ndimage.label(frontier, structure=np.ones((3, 3), bool))
    best = None
    for label in range(1, count + 1):
        members = labels == label
        if members.sum() < min_cells:
            continue
        reach = np.where(members & (steps >= 0), steps, np.iinfo(np.int32).max)
        index = int(reach.argmin())
        if reach.flat[index] == np.iinfo(np.int32).max:
            continue
        if best is None or reach.flat[index] < best[0]:
            best = reach.flat[index], index, members
    if best is None:
        return None

    _, index, members = best
    goal = divmod(index, cols)
    near = ndimage.binary_dilation(members, _disk(3)) & unknown
    ur, uc = np.nonzero(near)
    look = math.atan2(ur.mean() - goal[0], uc.mean() - goal[1]) if ur.size else 0.0
    return Goal(_corners(grid, passable, parent, start_cell, goal), look)


def plan_view(grid: Grid, start: tuple[float, float], target: tuple[float, float],
              clearance: float, distance: float,
              skip: Sequence[float] = ()) -> Goal | None:
    """Path to the nearest reachable spot about distance from target with a clear view of it.

    Bearings within APART of one in skip are left out, so looking again means
    looking from another side.
    """
    start_cell = grid.cell(*start)
    if not _inside(grid, start_cell):
        return None
    passable = passable_cells(grid, start_cell, clearance)
    steps, parent = distances(passable, start_cell)
    best = None
    for radius in (distance, distance * 0.7, distance * 1.4):
        for angle in np.linspace(0, math.tau, 24, endpoint=False):
            if any(abs(math.remainder(angle - other, math.tau)) < APART for other in skip):
                continue
            spot = (target[0] + radius * math.cos(angle), target[1] + radius * math.sin(angle))
            cell = grid.cell(*spot)
            if not _inside(grid, cell) or steps[cell] < 0:
                continue
            if grid.sees(spot, target) and (best is None or steps[cell] < best[0]):
                best = steps[cell], cell, spot
        if best is not None:
            break
    if best is None:
        return None
    _, cell, spot = best
    path = _corners(grid, passable, parent, start_cell, cell) if cell != start_cell else []
    return Goal(path, math.atan2(target[1] - spot[1], target[0] - spot[0]))


def plan_path(grid: Grid, start: tuple[float, float], goal: tuple[float, float],
              clearance: float, look_yaw: float) -> Goal | None:
    """Path to a given point, or None when it cannot be reached."""
    start_cell, goal_cell = grid.cell(*start), grid.cell(*goal)
    if not (_inside(grid, start_cell) and _inside(grid, goal_cell)):
        return None
    passable = passable_cells(grid, start_cell, clearance)
    steps, parent = distances(passable, start_cell)
    if steps[goal_cell] < 0:
        return None
    path = _corners(grid, passable, parent, start_cell, goal_cell) if goal_cell != start_cell else []
    return Goal(path, look_yaw)


def leg(flight: Flight, goal: Goal) -> bool | None:
    """Fly at most LEG toward the goal. True on reaching it, False part way, None on failure."""
    if not goal.path:
        return True
    here = flight.here()
    x, y = goal.path[0]
    distance = math.dist(here, (x, y))
    if distance > LEG:
        x, y = (here[0] + (x - here[0]) * LEG / distance,
                here[1] + (y - here[1]) * LEG / distance)
    flight.get_logger().info(f"leg to map ({x:.1f}, {y:.1f}), goal at "
                             f"({goal.path[-1][0]:.1f}, {goal.path[-1][1]:.1f})")
    # Face the leg first: the obstacle scan only covers what the camera sees.
    heading = math.atan2(y - here[1], x - here[0])
    flight.turn(heading)
    if not flight.fly_to(x, y, heading):
        flight.get_logger().warn("leg timed out, giving up on that goal")
        return None
    return len(goal.path) == 1 and distance <= LEG


def travel(flight: Flight, replan: Callable[[Grid], Goal | None], max_legs: int = 12) -> bool:
    """Fly leg by leg, replanning on the latest map, and face look_yaw on arrival."""
    for _ in range(max_legs):
        goal = replan(flight.grid())
        if goal is None:
            return False
        arrived = leg(flight, goal)
        if arrived is None:
            return False
        if arrived:
            flight.turn(goal.look_yaw)
            return True
    return False


def look_at(flight: Flight, target: tuple[float, float], hover: float,
            skip: Sequence[float] = ()) -> float | None:
    """Go where target is in clear view, face it and hover.

    Returns the bearing it looked from, to pass as skip for a look from another
    side, or None when no viewpoint was reachable.
    """
    picked: Goal | None = None

    def replan(grid: Grid) -> Goal | None:
        nonlocal picked
        picked = plan_view(grid, flight.here(), target, CLEARANCE, INSPECT_RANGE, skip)
        return picked

    if not travel(flight, replan) or picked is None:
        return None
    flight.wait(lambda: False, hover)
    return math.remainder(picked.look_yaw + math.pi, math.tau)


class FrontierExplorer:
    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--max-time", type=float, default=600.0,
                            help="give up after this many seconds")
        parser.add_argument("--no-inspect", action="store_true",
                            help="do not detour to take a closer look at unsure people")

    def __init__(self, args: argparse.Namespace) -> None:
        self.max_time, self.inspect_candidates = args.max_time, not args.no_inspect
        self.inspections: dict[int, int] = {}
        self.coverage = Coverage(LOOK_RANGE, HALF_FOV)
        self.attempted: list[tuple[float, float]] = []

    def run(self, flight: Flight) -> None:
        log = flight.get_logger()
        flight.require(lambda: flight.map is not None, "/map")
        for _ in range(4):
            flight.turn(math.remainder(flight.heading() + math.pi / 2, math.tau))

        deadline = flight.get_clock().now().nanoseconds + int(self.max_time * 1e9)
        visited: list[tuple[float, float]] = []
        known, grew = 0, flight.get_clock().now().nanoseconds
        chasing, spent = None, 0
        while flight.get_clock().now().nanoseconds < deadline:
            self.look(flight)
            if self.inspect_candidates:
                self.inspect(flight)
            now = flight.get_clock().now().nanoseconds
            grid = flight.grid()
            if (fresh := int((grid.cells >= 0).sum())) > known + GROWTH:
                known, grew = fresh, now
            stale = now - grew > STALE * 1e9
            goal = None if stale else plan(grid, flight.here(), CLEARANCE, MIN_CELLS,
                                           visited, VISITED_RADIUS)
            if goal is None:
                # The map can be complete while most of it has never been in frame: a level
                # laser slice maps a hall from a few spots, a person is only found by looking.
                log.info(f"{'map has stopped growing' if stale else 'no reachable frontier left'}"
                         ", looking at what the camera has missed")
                if not self.look_at_gap(flight):
                    log.info("nothing left unlooked at")
                    break
                continue
            # Cells beyond a wall are mapped through gaps in it and look reachable, so a
            # frontier the drone keeps aiming at without arriving is one it cannot have.
            target = goal.path[-1]
            if chasing is None or math.dist(chasing, target) > VISITED_RADIUS:
                chasing, spent = target, 0
            spent += 1
            if spent > GIVE_UP:
                log.info(f"giving up on the frontier at ({target[0]:.1f}, {target[1]:.1f})")
                visited.append(target)
                chasing = None
                continue
            arrived = leg(flight, goal)
            if arrived is None:
                visited.append(target)
                chasing = None
            elif arrived:
                flight.turn(goal.look_yaw)
                visited.append(flight.here())
                chasing = None
        else:
            log.info(f"stopped after {self.max_time:.0f} s")
        if self.inspect_candidates:
            self.inspect(flight)
        log.info("holding")

    def look(self, flight: Flight) -> None:
        """Remember what the camera has in view from where it is now."""
        self.coverage.mark(flight.grid(), flight.here(), flight.heading())

    def look_at_gap(self, flight: Flight) -> bool:
        """Go and look at the mapped place the camera has never had in view that is worth most."""
        grid = flight.grid()
        here = flight.here()
        gaps = self.coverage.unseen(grid)
        rows, cols = np.mgrid[0:gaps.shape[0], 0:gaps.shape[1]]
        for x, y in self.attempted:
            r, c = grid.cell(x, y)
            gaps &= (rows - r) ** 2 + (cols - c) ** 2 > (VISITED_RADIUS / grid.resolution) ** 2

        labels, count = ndimage.label(gaps, structure=np.ones((3, 3), bool))
        sizes = np.bincount(labels.ravel(), minlength=count + 1)
        worth = gaps & (sizes[labels] >= MIN_LOOK)
        # Only what can be looked at from somewhere the drone can actually reach. Free cells
        # beyond a wall are mapped through doorways and gaps and would otherwise be chased.
        start_cell = grid.cell(*here)
        steps, _ = distances(passable_cells(grid, start_cell, CLEARANCE), start_cell)
        worth &= ndimage.binary_dilation(steps >= 0, _disk(round(INSPECT_RANGE / grid.resolution)))
        if not worth.any():
            return False

        # The gap worth the trip is the one where a look uncovers most for the flying it costs,
        # not the nearest cell: one look covers a cone, so chasing the nearest crawls.
        span = round(LOOK_RANGE / grid.resolution)
        crowd = ndimage.uniform_filter(worth.astype(np.float32), size=span)
        xs, ys = grid.point(rows, cols)
        score = np.where(worth, crowd / (1 + np.hypot(xs - here[0], ys - here[1])), 0.0)
        cell = np.unravel_index(int(score.argmax()), score.shape)
        target = grid.point(*cell)
        flight.get_logger().info(f"looking at ({target[0]:.1f}, {target[1]:.1f}), "
                                 f"{int(worth.sum()) * grid.resolution ** 2:.0f} m2 never in view")
        # Whether it worked or no viewpoint was reachable, that gap has had its trip.
        self.attempted.append(target)
        look_at(flight, target, LOOK_HOVER)
        self.look(flight)
        return True

    def inspect(self, flight: Flight) -> None:
        """Look at unsure candidates, nearest first, until none is left worth a look."""
        while True:
            here = flight.here()
            unsure = [c for c in flight.people()
                      if c.status == "candidate" and c.confidence < INSPECT_BELOW
                      and self.inspections.get(c.id, 0) < MAX_INSPECTIONS]
            if not unsure:
                return
            candidate = min(unsure, key=lambda c: math.dist(here, (c.x, c.y)))
            self.inspections[candidate.id] = self.inspections.get(candidate.id, 0) + 1
            flight.get_logger().info(
                f"inspecting candidate {candidate.id} at ({candidate.x:.1f}, "
                f"{candidate.y:.1f}), confidence {candidate.confidence:.2f}")
            if look_at(flight, (candidate.x, candidate.y), INSPECT_HOVER) is None:
                flight.get_logger().warn("no clear view of it")
