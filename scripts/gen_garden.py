#!/usr/bin/env python3
"""Generate the garden world: a house, a fenced garden with trees and hedges, people outside.

The second scenario, to show the flight and perception code does not know the
warehouse. Nothing here is referenced from the code: pass world:=garden.

    ./scripts/gen_garden.py
    ./run.sh sim world:=garden
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

from worldgen import Box, ground, static_model, targets, world, write

PLOT = (32.0, 22.0)      # fenced garden, the drone spawns at (-13.5, 0) inside it
HEDGE_H = 3.2            # boundary hedge, tall enough to be an obstacle at flight height
FENCE_H = 1.2

GREEN = (0.22, 0.42, 0.18)
DARK_GREEN = (0.16, 0.32, 0.14)
BARK = (0.32, 0.24, 0.16)
WOOD = (0.62, 0.48, 0.30)
BRICK = (0.62, 0.36, 0.28)
ROOF = (0.35, 0.28, 0.26)
GLASS = (0.55, 0.68, 0.72)
PETALS = ((0.80, 0.30, 0.35), (0.85, 0.65, 0.20), (0.60, 0.35, 0.70))

HOUSE = (12.0, 8.0, 5.0)
HOUSE_AT = (8.0, 5.0)

# Trees and free-standing hedges, the things a person can stand behind. A hedge
# is under the 2.3 m scan slice, so it is missing from the 2D map: the drone has
# to notice from the depth image that the view is blocked.
TREES = ((-6.0, -6.5), (1.0, 6.0), (-10.5, 5.5), (10.0, -6.0), (4.0, -8.5))
HEDGES = (
    (-2.0, 1.5, 7.0, 0.8, 0.0),
    (6.0, -3.0, 6.0, 0.8, 1.5708),
    (-9.5, -2.0, 4.0, 0.8, 1.5708),
)

# People outside in the garden: on the open lawn, behind a hedge, behind the
# shed, against the house wall, under a tree and in the far corner.
TARGET_SPOTS = (
    (-5.0, 0.0, 0.0, "lawn"),
    (9.0, -1.0, 3.14, "lawn"),
    (-2.5, 3.2, -1.57, "behind hedge"),
    (-8.3, -2.0, 1.0, "behind hedge"),
    (-6.3, -8.0, 0.5, "under tree"),
    (13.3, -9.2, 2.2, "corner"),
)

SCENE = """    <scene>
      <ambient>0.55 0.55 0.52 1</ambient>
      <background>0.55 0.68 0.85 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>"""

SUN = """    <light name="sun" type="directional">
      <pose>0 0 12 0 0 0</pose>
      <direction>-0.4 0.3 -0.9</direction>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>false</cast_shadows>
    </light>"""


def boundary() -> str:
    """A picket fence with a hedge grown up behind it, all the way round."""
    lx, ly = PLOT
    thick = 0.7
    boxes = []
    for name, (x, y), (sx, sy) in (("north", (0, ly / 2 - thick / 2), (lx, thick)),
                                   ("south", (0, -ly / 2 + thick / 2), (lx, thick)),
                                   ("east", (lx / 2 - thick / 2, 0), (thick, ly)),
                                   ("west", (-lx / 2 + thick / 2, 0), (thick, ly))):
        boxes.append(Box(f"hedge_{name}", (x, y, HEDGE_H / 2), (sx, sy, HEDGE_H), DARK_GREEN))
    # A post every 2 m along the fence line, with two rails between them.
    for i, x in enumerate(_along(lx)):
        for sy in (-1, 1):
            boxes.append(Box(f"post_x{i}_{'n' if sy > 0 else 's'}", (x, sy * ly / 2, FENCE_H / 2),
                             (0.09, 0.09, FENCE_H), WOOD))
    for j, y in enumerate(_along(ly)):
        for sx in (-1, 1):
            boxes.append(Box(f"post_y{j}_{'e' if sx > 0 else 'w'}", (sx * lx / 2, y, FENCE_H / 2),
                             (0.09, 0.09, FENCE_H), WOOD))
    for level, z in enumerate((0.45, 1.0)):
        for sy in (-1, 1):
            boxes.append(Box(f"rail_x{level}_{'n' if sy > 0 else 's'}", (0, sy * ly / 2, z),
                             (lx, 0.06, 0.12), WOOD, collide=False))
        for sx in (-1, 1):
            boxes.append(Box(f"rail_y{level}_{'e' if sx > 0 else 'w'}", (sx * lx / 2, 0, z),
                             (0.06, ly, 0.12), WOOD, collide=False))
    return static_model("boundary", (0, 0, 0), boxes)


def _along(length: float, step: float = 2.0) -> list[float]:
    return [-length / 2 + i * step for i in range(int(length / step) + 1)]


def house() -> str:
    w, d, h = HOUSE
    boxes = [
        Box("walls", (0, 0, h / 2), (w, d, h), BRICK),
        Box("roof", (0, 0, h + 0.5), (w + 0.8, d + 0.8, 1.0), ROOF),
        Box("chimney", (w / 2 - 1.5, 0, h + 1.8), (0.8, 0.8, 1.6), BRICK),
    ]
    # Windows and a door: no collision, but the texture visual odometry needs.
    for i, x in enumerate((-4.0, -1.5, 1.5, 4.0)):
        for level, z in enumerate((1.6, 3.6)):
            boxes.append(Box(f"window_{i}_{level}", (x, -d / 2 - 0.02, z), (1.3, 0.06, 1.2),
                             GLASS, collide=False))
    boxes.append(Box("door", (0, -d / 2 - 0.02, 1.05), (1.1, 0.06, 2.1), (0.35, 0.22, 0.16),
                     collide=False))
    boxes.append(Box("terrace", (0, -d / 2 - 2.0, 0.05), (w, 4.0, 0.1), (0.72, 0.70, 0.66)))
    return static_model("house", (*HOUSE_AT, 0.0), boxes)


def tree(name: str, x: float, y: float, rng: random.Random) -> str:
    trunk = rng.uniform(2.0, 2.6)
    spread = rng.uniform(2.8, 4.0)
    boxes = [Box("trunk", (0, 0, trunk / 2), (0.36, 0.36, trunk), BARK)]
    # Three slabs of leaves, the widest in the middle, up to about 5 m.
    for level, (dz, scale) in enumerate(((0.0, 0.75), (0.9, 1.0), (1.8, 0.6))):
        boxes.append(Box(f"canopy_{level}", (0, 0, trunk + 0.4 + dz),
                         (spread * scale, spread * scale, 1.0), GREEN))
    return static_model(name, (x, y, 0.0), boxes)


def hedges() -> list[str]:
    return [static_model(f"hedge_row_{i}", (x, y, 0.0),
                         [Box("bush", (0, 0, 0.9), (length, thick, 1.8), GREEN, yaw=yaw)])
            for i, (x, y, length, thick, yaw) in enumerate(HEDGES)]


def furniture(rng: random.Random) -> list[str]:
    shed = static_model("shed", (11.0, -7.5, 0.0), [
        Box("body", (0, 0, 1.1), (3.0, 2.4, 2.2), WOOD),
        Box("roof", (0, 0, 2.3), (3.4, 2.8, 0.2), ROOF),
    ])
    table = static_model("garden_table", (4.0, -0.5, 0.0), [
        Box("top", (0, 0, 0.75), (1.6, 0.9, 0.08), WOOD),
        *[Box(f"leg_{i}", (sx * 0.7, sy * 0.35, 0.37), (0.08, 0.08, 0.75), WOOD)
          for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))],
        *[Box(f"chair_{i}", (sx * 1.2, 0, 0.45), (0.5, 0.5, 0.9), WOOD, yaw=0.3 * sx)
          for i, sx in enumerate((-1, 1))],
    ])
    greenhouse = static_model("greenhouse", (-12.0, 7.5, 0.0), [
        Box("frame", (0, 0, 1.15), (3.0, 2.2, 2.3), GLASS),
        Box("ridge", (0, 0, 2.35), (3.2, 2.4, 0.12), WOOD),
    ])
    beds = static_model("flower_beds", (0, 0, 0.0), [
        Box(f"bed_{i}", (x, y, 0.06), (2.4, 1.2, 0.12), rng.choice(PETALS),
            collide=False, yaw=rng.uniform(0, 1.5))
        for i, (x, y) in enumerate(((-13.0, -6.0), (-3.0, 8.5), (12.0, 2.0), (-8.0, 9.0)))
    ])
    # Mown stripes, so the lawn is not a blank plane for visual odometry.
    stripes = static_model("lawn_stripes", (0, 0, 0.0), [
        Box(f"stripe_{i}", (-15 + 2.5 * i + 1.25, 0, 0.005), (1.25, PLOT[1], 0.01),
            (0.26, 0.48, 0.20), collide=False)
        for i in range(int(PLOT[0] / 2.5))
    ])
    return [shed, table, greenhouse, beds, stripes]


def build(seed: int) -> str:
    rng = random.Random(seed)
    trees = [tree(f"tree_{i}", x, y, rng) for i, (x, y) in enumerate(TREES)]
    people, _ = targets(TARGET_SPOTS)
    models = "\n\n".join([ground((0.30, 0.50, 0.24), (0.20, 0.34, 0.16)), boundary(), house(),
                          *trees, *hedges(), *furniture(rng), *people])
    return world("garden", "gen_garden.py", seed, SCENE, SUN, models)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "worlds" / "garden.sdf",
    )
    args = parser.parse_args()
    write(args.output, build(args.seed), TARGET_SPOTS)


if __name__ == "__main__":
    main()
