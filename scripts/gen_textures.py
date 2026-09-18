#!/usr/bin/env python3
"""Generate the surface textures the worlds paint onto walls, floors and ground.

A flat colour is the worst case a camera can be given: visual odometry needs
corners to track, and a blank wall has none, so the drone loses its position
estimate the moment nothing else is in frame. A real warehouse wall is not blank
- it has grain, stains, panel seams and scuffs - so the sim was being harder than
reality rather than easier.

Each texture is noise over several octaves, which puts detail at every scale, so
it still gives the tracker corners whether the camera is a metre from the wall or
eight. The seams are deliberately irregular: evenly spaced marks all look alike
and a feature matcher will happily pair the wrong two.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

TEXTURES = Path(__file__).resolve().parent.parent / "models" / "surfaces" / "materials" / "textures"
SIZE = 2048               # px, about 1 cm per pixel stretched over a 20 m wall
COARSEST = 8              # cells across the finest and coarsest octave: below 8 the
FINEST = 512              # noise is a gradient, above 512 it aliases into mush


def octaves(rng: np.random.Generator, size: int) -> np.ndarray:
    """Noise summed over halving scales.

    Amplitude falls as the square root of the scale rather than the scale itself.
    Dropping it faster looks more like real stone but leaves almost all the
    energy in the coarsest octave, and a smooth gradient has no corners in it.
    """
    total, weight, cells = np.zeros((size, size), np.float32), 0.0, COARSEST
    while cells <= FINEST:
        scale = 1.0 / math.sqrt(cells)
        total += scale * cv2.resize(rng.random((cells, cells), np.float32), (size, size),
                                    interpolation=cv2.INTER_CUBIC)
        weight += scale
        cells *= 2
    return total / weight


def seams(rng: np.random.Generator, size: int, count: int, width: int) -> np.ndarray:
    """Darker scratches and panel edges, each a segment rather than a full line.

    Lines spanning the whole surface would cross into a regular lattice, and one
    intersection of a lattice looks exactly like the next.
    """
    marks = np.zeros((size, size), np.float32)
    for _ in range(count):
        start = rng.integers(0, size, 2)
        length = rng.integers(size // 12, size // 3)
        end = start + (np.array([length, 0]) if rng.random() < 0.5 else np.array([0, length]))
        cv2.line(marks, tuple(start), tuple(end), 1.0, width)
    return cv2.GaussianBlur(marks, (0, 0), width)


def surface(rng: np.random.Generator, colour: tuple[float, float, float],
            contrast: float, seam_count: int) -> np.ndarray:
    grain = octaves(rng, SIZE)
    grain = (grain - grain.mean()) / grain.std()
    shade = 1.0 + contrast * grain - 0.35 * seams(rng, SIZE, seam_count, 5)
    bgr = np.clip(np.asarray(colour[::-1], np.float32) * shade[..., None], 0.0, 1.0)
    return (bgr * 255).astype(np.uint8)


SURFACES = {
    # name: colour, how far the grain swings, how many seams
    "concrete": ((0.72, 0.72, 0.70), 0.16, 60),
    "floor": ((0.45, 0.46, 0.48), 0.12, 90),
    "grass": ((0.35, 0.50, 0.28), 0.24, 0),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    seed = parser.parse_args().seed

    TEXTURES.mkdir(parents=True, exist_ok=True)
    for i, (name, (colour, contrast, seam_count)) in enumerate(SURFACES.items()):
        rng = np.random.default_rng(seed + i)
        path = TEXTURES / f"{name}.png"
        cv2.imwrite(str(path), surface(rng, colour, contrast, seam_count))
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
