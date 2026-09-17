"""Frontier exploration on an occupancy grid: where to fly next, and how to get there.

A frontier is known free space next to unknown space. Flying to the nearest one
and looking into the unknown grows the map until no reachable frontier is left.
plan() and plan_view() are the planner on its own; FrontierExplorer flies them.

Between legs it also takes a closer look at candidates, people the tracker is not
sure of yet: it flies to a spot a few metres away with a clear view, faces them
and hovers. A real person keeps being detected and gets confirmed, a false one
stops being detected and loses confidence.
"""

from __future__ import annotations

import argparse
import math
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

from occupancy import OCCUPIED, UNKNOWN, Grid

if TYPE_CHECKING:
    from flight import Flight

# Legs are short so each plan uses the map the last leg revealed.
LEG = 3.0
CLEARANCE = 0.8          # arm tip is 0.4 m from the centre, the rest is position error
MIN_CELLS = 4
VISITED_RADIUS = 1.5     # a frontier still unknown after looking at it is given up

# Candidates: people the tracker is not sure of yet.
INSPECT_BELOW = 0.9      # confidence under which a candidate is worth a detour
INSPECT_RANGE = 3.5      # from where it is looked at, well inside the tracker's miss range
INSPECT_HOVER = 3.0      # s of looking, ~12 detector frames
MAX_INSPECTIONS = 2
ARRIVED = 0.5

NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Goal:
    path: list[tuple[float, float]]  # waypoints after the start, the goal last
    look_yaw: float                  # where to face on arrival


def _disk(radius: int) -> np.ndarray:
    r = np.arange(-radius, radius + 1)
    return r[:, None] ** 2 + r[None, :] ** 2 <= radius ** 2


def _distances(passable: np.ndarray, start: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
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


def _passable(grid: Grid, start_cell: tuple[int, int], clearance: float) -> np.ndarray:
    free = (grid.cells >= 0) & (grid.cells < OCCUPIED)
    margin = math.ceil(clearance / grid.resolution)
    passable = free & ~ndimage.binary_dilation(grid.cells >= OCCUPIED, _disk(margin))
    # The drone may hover inside the clearance margin; it still has to leave from there.
    escape = np.zeros_like(passable)
    escape[max(start_cell[0] - margin, 0):start_cell[0] + margin + 1,
           max(start_cell[1] - margin, 0):start_cell[1] + margin + 1] = True
    passable |= escape & free
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
    passable = _passable(grid, start_cell, clearance)
    unknown = grid.cells == UNKNOWN
    rows, cols = grid.cells.shape

    frontier = passable & ndimage.binary_dilation(unknown, np.ones((3, 3), bool))
    ys, xs = np.mgrid[0:rows, 0:cols]
    for x, y in avoid:
        r, c = grid.cell(x, y)
        frontier &= (ys - r) ** 2 + (xs - c) ** 2 > (avoid_radius / grid.resolution) ** 2

    steps, parent = _distances(passable, start_cell)
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
              clearance: float, distance: float) -> Goal | None:
    """Path to the nearest reachable spot about distance from target with a clear view of it."""
    start_cell = grid.cell(*start)
    if not _inside(grid, start_cell):
        return None
    passable = _passable(grid, start_cell, clearance)
    steps, parent = _distances(passable, start_cell)
    best = None
    for radius in (distance, distance * 0.7, distance * 1.4):
        for angle in np.linspace(0, math.tau, 24, endpoint=False):
            spot = (target[0] + radius * math.cos(angle), target[1] + radius * math.sin(angle))
            cell = grid.cell(*spot)
            if not _inside(grid, cell) or steps[cell] < 0:
                continue
            # Stop short of the target, whose own cell may be marked occupied by its body.
            end = (target[0] - 0.6 * math.cos(angle), target[1] - 0.6 * math.sin(angle))
            if grid.clear(spot, end) and (best is None or steps[cell] < best[0]):
                best = steps[cell], cell, spot
        if best is not None:
            break
    if best is None:
        return None
    _, cell, spot = best
    path = _corners(grid, passable, parent, start_cell, cell) if cell != start_cell else []
    return Goal(path, math.atan2(target[1] - spot[1], target[0] - spot[0]))

class FrontierExplorer:
    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--max-time", type=float, default=600.0,
                            help="give up after this many seconds")
        parser.add_argument("--no-inspect", action="store_true",
                            help="do not detour to take a closer look at unsure people")

    def __init__(self, args: argparse.Namespace) -> None:
        self.max_time, self.inspect_candidates = args.max_time, not args.no_inspect
        self.inspections: dict[str, int] = {}

    def run(self, flight: Flight) -> None:
        log = flight.get_logger()
        flight.require(lambda: flight.map is not None, "/map")
        for _ in range(4):
            flight.turn(math.remainder(flight.heading() + math.pi / 2, math.tau))

        deadline = flight.get_clock().now().nanoseconds + int(self.max_time * 1e9)
        visited: list[tuple[float, float]] = []
        while flight.get_clock().now().nanoseconds < deadline:
            if self.inspect_candidates:
                self.inspect(flight)
            goal = plan(flight.grid(), flight.here(), CLEARANCE, MIN_CELLS, visited, VISITED_RADIUS)
            if goal is None:
                log.info("no reachable frontier left")
                break
            arrived = self.leg(flight, goal)
            if arrived is None:
                visited.append(goal.path[-1])
            elif arrived:
                flight.turn(goal.look_yaw)
                visited.append(flight.here())
        else:
            log.info(f"stopped after {self.max_time:.0f} s")
        if self.inspect_candidates:
            self.inspect(flight)
        log.info("holding")

    def leg(self, flight: Flight, goal: Goal) -> bool | None:
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

    def inspect(self, flight: Flight) -> None:
        """Look at unsure candidates, nearest first, until none is left worth a look."""
        log = flight.get_logger()
        while True:
            here = flight.here()
            unsure = [c for c in flight.candidates()
                      if c.confidence < INSPECT_BELOW
                      and self.inspections.get(c.id, 0) < MAX_INSPECTIONS]
            if not unsure:
                return
            candidate = min(unsure, key=lambda c: math.dist(here, (c.x, c.y)))
            self.inspections[candidate.id] = self.inspections.get(candidate.id, 0) + 1
            log.info(f"inspecting candidate {candidate.id} at ({candidate.x:.1f}, "
                     f"{candidate.y:.1f}), confidence {candidate.confidence:.2f}")
            for _ in range(10):
                goal = plan_view(flight.grid(), flight.here(), (candidate.x, candidate.y),
                                 CLEARANCE, INSPECT_RANGE)
                if goal is None:
                    log.warn("no clear view of it")
                    break
                arrived = self.leg(flight, goal)
                if arrived is None:
                    break
                if arrived:
                    flight.turn(goal.look_yaw)
                    flight.wait(lambda: False, INSPECT_HOVER)
                    break
