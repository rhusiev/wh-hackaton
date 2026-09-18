#!/usr/bin/env python3
"""Generate the warehouse world: racks in rows inside a closed hall."""

from __future__ import annotations

import argparse
import random
from pathlib import Path

from worldgen import Box, ground, static_model, targets, world, write

HALL = (32.0, 20.0, 8.0)
WALL_T = 0.2
WALL = "concrete"

RACK = (2.6, 1.1, 4.0)
DECK_Z = (0.25, 1.35, 2.45, 3.55)
ROW_Y = (-7.5, -2.5, 2.5, 7.5)
RACK_X = (-10.4, -5.2, 0.0, 5.2, 10.4)

CARGO_COLOURS = (
    (0.62, 0.45, 0.28),
    (0.70, 0.52, 0.32),
    (0.55, 0.38, 0.22),
    (0.30, 0.42, 0.62),
    (0.66, 0.24, 0.22),
    (0.28, 0.52, 0.36),
)

# Search targets. Aisle spots are visible down a lane; the rest sit in the 2.6 m
# gaps between racks, which is the case the AR overlay exists for - the operator
# cannot see them, the drone flying the next aisle over can.
TARGET_SPOTS = (
    (-9.0, 0.0, 0.0, "aisle"),
    (4.5, 5.0, 3.14, "aisle"),
    (-2.6, -6.6, 1.57, "occluded"),
    (7.8, 3.6, -1.57, "occluded"),
    (-7.8, -1.4, 1.57, "occluded"),
    (12.5, -7.0, 0.8, "corner"),
)

SCENE = """    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.2 0.22 0.25 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>"""


def rack_boxes(rng: random.Random) -> list[Box]:
    w, d, h = RACK
    steel = (0.25, 0.32, 0.45)
    boxes = [
        Box(f"upright_{i}", (sx * (w / 2 - 0.05), sy * (d / 2 - 0.05), h / 2),
            (0.1, 0.1, h), steel)
        for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))
    ]
    for level, z in enumerate(DECK_Z):
        boxes.append(Box(f"deck_{level}", (0, 0, z), (w, d, 0.07), (0.85, 0.55, 0.12)))
        slots = rng.sample(range(3), k=rng.randint(1, 3))
        for slot in slots:
            cw = rng.uniform(0.55, 0.75)
            ch = rng.uniform(0.45, 0.8)
            boxes.append(Box(
                f"cargo_{level}_{slot}",
                ((slot - 1) * (w / 3), rng.uniform(-0.1, 0.1), z + 0.035 + ch / 2),
                (cw, rng.uniform(0.6, 0.85), ch),
                rng.choice(CARGO_COLOURS),
                yaw=rng.uniform(-0.15, 0.15),
            ))
    return boxes


def shell() -> str:
    lx, ly, lz = HALL
    grey = (0.72, 0.72, 0.70)
    walls = [
        Box("wall_north", (0, ly / 2, lz / 2), (lx + 2 * WALL_T, WALL_T, lz), grey, texture=WALL),
        Box("wall_south", (0, -ly / 2, lz / 2), (lx + 2 * WALL_T, WALL_T, lz), grey, texture=WALL),
        Box("wall_east", (lx / 2, 0, lz / 2), (WALL_T, ly, lz), grey, texture=WALL),
        Box("wall_west", (-lx / 2, 0, lz / 2), (WALL_T, ly, lz), grey, texture=WALL),
        Box("roof", (0, 0, lz), (lx + 2 * WALL_T, ly + 2 * WALL_T, WALL_T), (0.35, 0.36, 0.38),
            texture=WALL),
    ]
    # Painted bands on top of the concrete: the texture carries the fine detail,
    # these are the landmarks that tell one stretch of wall from another.
    for i, x in enumerate(range(-14, 15, 4)):
        for sy in (-1, 1):
            walls.append(Box(
                f"band_{i}_{'n' if sy > 0 else 's'}",
                (x, sy * (ly / 2 - WALL_T / 2 - 0.01), 2.0),
                (0.6, 0.02, 3.0),
                CARGO_COLOURS[i % len(CARGO_COLOURS)],
                collide=False,
            ))
    return static_model("hall", (0, 0, 0), walls)


def floor_markings() -> str:
    boxes = []
    for i, y in enumerate((-5.0, 0.0, 5.0)):
        for sy in (-1, 1):
            boxes.append(Box(
                f"aisle_line_{i}_{sy}",
                (0, y + sy * 1.7, 0.005),
                (26.0, 0.12, 0.01),
                (0.95, 0.85, 0.15),
                collide=False,
            ))
    return static_model("floor_markings", (0, 0, 0), boxes)


def obstacles(rng: random.Random) -> list[str]:
    models = []
    spots = [
        (-8.0, 0.0), (-3.0, 5.0), (2.5, -5.0), (7.5, 0.0), (11.0, 5.0), (-11.5, -5.0),
    ]
    for i, (x, y) in enumerate(spots):
        h = rng.uniform(0.9, 1.6)
        models.append(static_model(
            f"pallet_stack_{i}", (x, y, 0.0),
            [
                Box("pallet", (0, 0, 0.07), (1.2, 0.8, 0.14), (0.55, 0.42, 0.26)),
                Box("load", (0, 0, 0.14 + h / 2), (1.05, 0.72, h), rng.choice(CARGO_COLOURS)),
            ],
        ))
    return models


def lights() -> str:
    out = []
    for i, x in enumerate((-12.0, -4.0, 4.0, 12.0)):
        for j, y in enumerate((-6.0, 0.0, 6.0)):
            out.append(f"""    <light name="bay_light_{i}_{j}" type="point">
      <pose>{x} {y} 6.8 0 0 0</pose>
      <diffuse>0.85 0.85 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>18</range><constant>0.4</constant><linear>0.05</linear><quadratic>0.004</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")
    return "\n".join(out)


def build(seed: int) -> str:
    rng = random.Random(seed)
    racks = [
        static_model(f"rack_{ri}_{ci}", (x, y, 0.0), rack_boxes(rng))
        for ri, y in enumerate(ROW_Y)
        for ci, x in enumerate(RACK_X)
    ]
    people, _ = targets(TARGET_SPOTS)
    models = "\n\n".join([ground((1.0, 1.0, 1.0), "floor"), shell(),
                          floor_markings(), *racks, *obstacles(rng), *people])
    return world("warehouse", "gen_warehouse.py", seed, SCENE, lights(), models)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "worlds" / "warehouse.sdf",
    )
    args = parser.parse_args()
    write(args.output, build(args.seed), TARGET_SPOTS)


if __name__ == "__main__":
    main()
