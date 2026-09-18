"""The 2D occupancy grid, as the planner and the tracker read /map."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

UNKNOWN = -1
OCCUPIED = 50

type Index = int | np.ndarray        # one cell, or a whole array of them at once


@dataclass
class Grid:
    cells: np.ndarray  # rows along y, -1 unknown, 0-100 occupancy
    resolution: float
    origin: tuple[float, float]

    @classmethod
    def from_msg(cls, msg) -> Grid:
        info = msg.info
        return cls(np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width),
                   info.resolution, (info.origin.position.x, info.origin.position.y))

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (int((y - self.origin[1]) / self.resolution),
                int((x - self.origin[0]) / self.resolution))

    def point(self, row: Index, col: Index) -> tuple[Index, Index]:
        """The middle of a cell, or of every cell of a pair of index arrays."""
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def clear(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """Known free all the way from a to b; unknown or outside counts as blocked."""
        n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / (self.resolution / 2)))
        rows = ((np.linspace(a[1], b[1], n) - self.origin[1]) / self.resolution).astype(int)
        cols = ((np.linspace(a[0], b[0], n) - self.origin[0]) / self.resolution).astype(int)
        inside = ((rows >= 0) & (rows < self.cells.shape[0])
                  & (cols >= 0) & (cols < self.cells.shape[1]))
        if not inside.all():
            return False
        cells = self.cells[rows, cols]
        return bool(((cells >= 0) & (cells < OCCUPIED)).all())

    def sees(self, eye: tuple[float, float], target: tuple[float, float]) -> bool:
        """A clear line up to 0.6 m short of target, whose own cells may be its body."""
        distance = math.dist(eye, target)
        keep = max(0.0, 1 - 0.6 / distance) if distance else 0.0
        return self.clear(eye, (eye[0] + (target[0] - eye[0]) * keep,
                                eye[1] + (target[1] - eye[1]) * keep))
