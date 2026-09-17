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
    margin: float = 0.1       # of the image, so a person cut by the edge is not expected

    def visible(self, xyz: np.ndarray, max_range: float) -> bool:
        x, y, z = self.rotation.T @ (xyz - self.position)
        if not 0.7 < z <= max_range:
            return False
        fx, fy, cx, cy = self.intrinsics
        u, v = fx * x / z + cx, fy * y / z + cy
        w, h = self.size
        if not (self.margin * w < u < (1 - self.margin) * w
                and self.margin * h < v < (1 - self.margin) * h):
            return False
        if self.grid is None:
            return False
        return self.grid.sees(tuple(self.position[:2]), tuple(xyz[:2]))
