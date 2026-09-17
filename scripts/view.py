"""Whether the camera, at one frame, should have seen something at a map position."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from occupancy import Grid

PATCH = 2          # pixels each way sampled around the point
BEHIND = 0.5       # m nearer than the point before it counts as blocked


@dataclass
class CameraView:
    rotation: np.ndarray      # camera optical frame -> map
    position: np.ndarray      # camera in the map frame
    intrinsics: tuple[float, float, float, float]  # fx, fy, cx, cy
    size: tuple[int, int]     # width, height in pixels
    grid: Grid | None
    depth: np.ndarray | None = None   # metres, the frame's own depth image
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
        if self.depth is not None:
            return self._unblocked(u / w, v / h, z)
        if self.grid is None:
            return False
        return self.grid.sees(tuple(self.position[:2]), tuple(xyz[:2]))

    def _unblocked(self, u: float, v: float, distance: float) -> bool:
        """Nothing measured in front of the point, from the depth image of this very frame.

        The 2D map cannot answer this: at flight height the line to a person
        runs over shelves and hedges that really block the view.
        """
        # Taken as fractions of the image: the depth stream may be scaled down.
        rows, cols = self.depth.shape
        r, c = int(v * rows), int(u * cols)
        patch = self.depth[max(r - PATCH, 0):r + PATCH + 1, max(c - PATCH, 0):c + PATCH + 1]
        near = patch[np.isfinite(patch) & (patch > 0)]
        return near.size == 0 or bool(np.median(near) > distance - BEHIND)
