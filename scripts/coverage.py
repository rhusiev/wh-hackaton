"""Where the camera has actually looked, kept beside the occupancy map.

The map says where the walls are. It says nothing about where the camera has
pointed, and the two finish at very different times: a level laser slice maps a
hall from a few positions, while a person is only found by looking at them. An
explorer that stops when the map is complete therefore stops while most of the
place has never been in frame.

Coverage answers "have we looked there". A cell counts as looked at once it has
been within range, inside the field of view, and with nothing occupied in
between. It is kept in its own half-metre cells in world coordinates, so it does
not care what the map's resolution or origin is, or whether they change.
"""

from __future__ import annotations

import numpy as np

from occupancy import OCCUPIED, Grid

CELL = 0.5               # m, the resolution coverage is remembered at
LOOK_RANGE = 7.0         # m, as far as a person is worth counting as looked at
HALF_FOV = 0.5           # rad, the camera's 0.6 with a margin for heading error
RAYS = 41                # across the field of view
KEY = 100000             # cells per row when a cell pair is packed into one int


class Coverage:
    """The cells the camera has had a clear view of."""

    def __init__(self, look_range: float, half_fov: float) -> None:
        self.look_range, self.half_fov = look_range, half_fov
        self.seen: set[int] = set()

    def mark(self, grid: Grid, position: tuple[float, float], heading: float) -> None:
        """Mark everything in view from here, stopping each ray at the first obstacle."""
        blocked = grid.cells >= OCCUPIED
        rows, cols = blocked.shape
        angles = heading + np.linspace(-self.half_fov, self.half_fov, RAYS)
        steps = np.arange(0, self.look_range, grid.resolution / 2)
        xs = position[0] + np.cos(angles)[:, None] * steps
        ys = position[1] + np.sin(angles)[:, None] * steps

        r = np.floor((ys - grid.origin[1]) / grid.resolution).astype(int)
        c = np.floor((xs - grid.origin[0]) / grid.resolution).astype(int)
        inside = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
        hit = np.zeros(xs.shape, dtype=bool)
        hit[inside] = blocked[r[inside], c[inside]]
        # Everything up to the first obstacle along a ray, and outside the map nothing.
        reach = np.cumsum(hit, axis=1) == 0
        keep = reach & inside
        self.seen.update(self._keys(xs[keep], ys[keep]).tolist())

    def unseen(self, grid: Grid) -> np.ndarray:
        """Mask of the grid's free cells that have never been in view."""
        free = (grid.cells >= 0) & (grid.cells < OCCUPIED)
        rows, cols = np.nonzero(free)
        xs, ys = grid.point(rows, cols)
        known = np.fromiter(self.seen, dtype=np.int64, count=len(self.seen))
        fresh = ~np.isin(self._keys(xs, ys), known)
        mask = np.zeros_like(free)
        mask[rows[fresh], cols[fresh]] = True
        return mask

    @staticmethod
    def _keys(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        return (np.floor(ys / CELL).astype(np.int64) * KEY
                + np.floor(xs / CELL).astype(np.int64))
