"""Whether the camera, at one frame, should have seen something at a map position."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from occupancy import Grid


@dataclass
class CameraView:
    rotation: np.ndarray      # camera optical frame -> map
    position: np.ndarray      # camera in the map frame
    intrinsics: tuple[float, float, float, float]  # fx, fy, cx, cy
    size: tuple[int, int]     # width, height in pixels
    grid: Grid | None
    max_range: float
    margin: float = 0.1       # of the image, so a person cut by the edge is not expected

    def visible(self, xyz: np.ndarray) -> bool:
        x, y, z = self.rotation.T @ (xyz - self.position)
        if not 0.7 < z <= self.max_range:
            return False
        fx, fy, cx, cy = self.intrinsics
        u, v = fx * x / z + cx, fy * y / z + cy
        w, h = self.size
        if not (self.margin * w < u < (1 - self.margin) * w
                and self.margin * h < v < (1 - self.margin) * h):
            return False
        if self.grid is None:
            return False
        # Stop short of the target, whose own cell may be marked occupied by its body.
        direction = xyz[:2] - self.position[:2]
        end = self.position[:2] + direction * max(0.0, 1 - 0.6 / np.linalg.norm(direction))
        return self.grid.clear(tuple(self.position[:2]), tuple(end))
