#!/usr/bin/env python3
"""Generate the warehouse world from primitives and the people in models/people."""

from __future__ import annotations

import argparse
import json
import random
from dataclasses import dataclass
from pathlib import Path

HALL = (32.0, 20.0, 8.0)
WALL_T = 0.2

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

ORIGIN_LATLON = (49.839700, 24.029700, 296.0)

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
# Fuel meshes (CC BY 4.0), feet at the origin, ~1.6 m tall.
PEOPLE = ("Nurse", "FemaleVisitor", "Scrubs")
PERSON_SIZE = (0.5, 0.4, 1.62)


@dataclass(frozen=True)
class Box:
    """A single box geometry placed inside a link."""

    name: str
    pose: tuple[float, float, float]
    size: tuple[float, float, float]
    colour: tuple[float, float, float]
    collide: bool = True
    yaw: float = 0.0

    def to_sdf(self, indent: str) -> str:
        x, y, z = self.pose
        sx, sy, sz = self.size
        r, g, b = self.colour
        pose = f"{x:.4g} {y:.4g} {z:.4g} 0 0 {self.yaw:.4g}"
        geom = f"<geometry><box><size>{sx:.4g} {sy:.4g} {sz:.4g}</size></box></geometry>"
        parts = [
            f'{indent}<visual name="{self.name}_v">',
            f"{indent}  <pose>{pose}</pose>",
            f"{indent}  {geom}",
            f"{indent}  <material>",
            f"{indent}    <ambient>{r * 0.5:.3g} {g * 0.5:.3g} {b * 0.5:.3g} 1</ambient>",
            f"{indent}    <diffuse>{r:.3g} {g:.3g} {b:.3g} 1</diffuse>",
            f"{indent}  </material>",
            f"{indent}</visual>",
        ]
        if self.collide:
            parts += [
                f'{indent}<collision name="{self.name}_c">',
                f"{indent}  <pose>{pose}</pose>",
                f"{indent}  {geom}",
                f"{indent}</collision>",
            ]
        return "\n".join(parts)


def static_model(name: str, pose: tuple[float, float, float], boxes: list[Box]) -> str:
    x, y, z = pose
    body = "\n".join(box.to_sdf("        ") for box in boxes)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4g} {y:.4g} {z:.4g} 0 0 0</pose>
      <link name="link">
{body}
      </link>
    </model>"""


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
        Box("wall_north", (0, ly / 2, lz / 2), (lx + 2 * WALL_T, WALL_T, lz), grey),
        Box("wall_south", (0, -ly / 2, lz / 2), (lx + 2 * WALL_T, WALL_T, lz), grey),
        Box("wall_east", (lx / 2, 0, lz / 2), (WALL_T, ly, lz), grey),
        Box("wall_west", (-lx / 2, 0, lz / 2), (WALL_T, ly, lz), grey),
        Box("roof", (0, 0, lz), (lx + 2 * WALL_T, ly + 2 * WALL_T, WALL_T), (0.35, 0.36, 0.38)),
    ]
    # Painted bands give feature-poor walls something for visual odometry to track.
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


def person(name: str, mesh: str, x: float, y: float, yaw: float) -> str:
    return f"""    <include>
      <uri>model://{mesh}</uri>
      <name>{name}</name>
      <pose>{x:.4g} {y:.4g} 0 0 0 {yaw:.4g}</pose>
    </include>"""


def targets() -> tuple[list[str], list[dict]]:
    models, truth = [], []
    for i, (x, y, yaw, kind) in enumerate(TARGET_SPOTS):
        name = f"person_{i}"
        models.append(person(name, PEOPLE[i % len(PEOPLE)], x, y, yaw))
        truth.append({"name": name, "label": "person", "kind": kind,
                      "xyz": [x, y, PERSON_SIZE[2] / 2], "size": list(PERSON_SIZE)})
    return models, truth


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
    lat, lon, alt = ORIGIN_LATLON
    people, _ = targets()
    models = "\n\n".join([shell(), floor_markings(), *racks, *obstacles(rng), *people])
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/gen_warehouse.py (seed {seed}). Do not edit by hand. -->
<sdf version="1.9">
  <world name="warehouse">

    <physics name="2ms" type="ignore">
      <!-- Lock-stepped with SITL, so each step is also a flight-controller tick.
           1 ms held the full stack to RTF 0.7; config/sitl.parm drops the loop
           to 250 Hz so the 500 Hz gyro clears its 1.8x pre-arm check. -->
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <!-- DART's default FCL checker alone holds this static hall to RTF 0.5. -->
      <dart><collision_detector>bullet</collision_detector></dart>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8066</gravity>
    <magnetic_field>2.0554e-05 0.0 4.5e-05</magnetic_field>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{alt}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.2 0.22 0.25 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

{lights()}

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.3 0.3 0.31 1</ambient>
            <diffuse>0.42 0.42 0.44 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{models}

  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "worlds" / "warehouse.sdf",
    )
    args = parser.parse_args()
    args.output.write_text(build(args.seed))

    # The detector stand-in needs to know where the targets really are.
    truth = args.output.with_name(f"{args.output.stem}_targets.json")
    _, entries = targets()
    truth.write_text(json.dumps({"world": args.output.stem, "targets": entries}, indent=2))
    print(f"wrote {args.output} and {truth}")


if __name__ == "__main__":
    main()
