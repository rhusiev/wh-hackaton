"""Frontier exploration on an occupancy grid: where to fly next, and how to get there.

A frontier is known free space next to unknown space. Flying to the nearest one
and looking into the unknown grows the map until no reachable frontier is left.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

UNKNOWN = -1
OCCUPIED = 50
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Grid:
    cells: np.ndarray  # rows along y, -1 unknown, 0-100 occupancy
    resolution: float
    origin: tuple[float, float]

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (int((y - self.origin[1]) / self.resolution),
                int((x - self.origin[0]) / self.resolution))

    def point(self, row: int, col: int) -> tuple[float, float]:
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)


@dataclass
class Goal:
    path: list[tuple[float, float]]  # waypoints after the start, the goal last
    look_yaw: float                  # faces the unknown space the frontier borders


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


def plan(grid: Grid, start: tuple[float, float], clearance: float, min_cells: int,
         avoid: list[tuple[float, float]], avoid_radius: float) -> Goal | None:
    """Path to the nearest reachable frontier, or None when the map is explored."""
    free = (grid.cells >= 0) & (grid.cells < OCCUPIED)
    unknown = grid.cells == UNKNOWN
    blocked = ndimage.binary_dilation(grid.cells >= OCCUPIED,
                                      _disk(math.ceil(clearance / grid.resolution)))
    passable = free & ~blocked
    start_cell = grid.cell(*start)
    rows, cols = grid.cells.shape
    if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols):
        return None
    # The drone may hover inside the clearance margin; it still has to leave from there.
    margin = math.ceil(clearance / grid.resolution)
    escape = np.zeros_like(passable)
    escape[max(start_cell[0] - margin, 0):start_cell[0] + margin + 1,
           max(start_cell[1] - margin, 0):start_cell[1] + margin + 1] = True
    passable |= escape & free
    passable[start_cell] = True

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
    chain = [divmod(index, cols)]
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

    goal = chain[-1]
    near = ndimage.binary_dilation(members, _disk(3)) & unknown
    ur, uc = np.nonzero(near)
    look = math.atan2(ur.mean() - goal[0], uc.mean() - goal[1]) if ur.size else 0.0
    return Goal([grid.point(*cell) for cell in corners], look)
