"""Shared pieces for the world generators: box primitives and the SDF frame around them.

A world is a list of static models made of boxes, plus people included from
models/people, plus the drone itself, plus the truth file the detector stand-in
and the scorer read.
Nothing in the flight code knows any of it; see gen_warehouse.py and
gen_garden.py for the two worlds built on this.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

WORLDS = Path(__file__).resolve().parent.parent / "worlds"
ORIGIN_LATLON = (49.839700, 24.029700, 296.0)
# Fuel meshes (CC BY 4.0), feet at the origin, ~1.6 m tall.
PEOPLE = ("Nurse", "FemaleVisitor", "Scrubs")
PERSON_SIZE = (0.5, 0.4, 1.62)
TEXTURE_URI = "model://surfaces/materials/textures"
DRONE = "tricopter"
DRONE_START = (-13.5, 0.0, 0.2, 0.0)    # x, y, z, yaw; config/gz_bridge.yaml uses the name too
# The AR glasses' camera, where ar_preview.py starts the wearer. It moves it from there.
WEARER = "wearer"
WEARER_START = (-15.5, 0.0, 1.7, 0.0)


def material(colour: tuple[float, float, float], texture: str | None, indent: str) -> list[str]:
    """A flat colour, or that colour already baked into a texture."""
    r, g, b = (1.0, 1.0, 1.0) if texture else colour
    lines = [
        f"{indent}<material>",
        f"{indent}  <ambient>{r * 0.5:.3g} {g * 0.5:.3g} {b * 0.5:.3g} 1</ambient>",
        f"{indent}  <diffuse>{r:.3g} {g:.3g} {b:.3g} 1</diffuse>",
    ]
    if texture:
        lines += [
            f"{indent}  <pbr><metal>",
            f"{indent}    <albedo_map>{TEXTURE_URI}/{texture}.png</albedo_map>",
            f"{indent}    <metalness>0</metalness><roughness>0.9</roughness>",
            f"{indent}  </metal></pbr>",
        ]
    return [*lines, f"{indent}</material>"]


@dataclass(frozen=True)
class Box:
    """A single box geometry placed inside a link."""

    name: str
    pose: tuple[float, float, float]
    size: tuple[float, float, float]
    colour: tuple[float, float, float]
    collide: bool = True
    yaw: float = 0.0
    texture: str | None = None      # a name under models/surfaces, see gen_textures.py

    def to_sdf(self, indent: str) -> str:
        x, y, z = self.pose
        sx, sy, sz = self.size
        pose = f"{x:.4g} {y:.4g} {z:.4g} 0 0 {self.yaw:.4g}"
        geom = f"<geometry><box><size>{sx:.4g} {sy:.4g} {sz:.4g}</size></box></geometry>"
        parts = [
            f'{indent}<visual name="{self.name}_v">',
            f"{indent}  <pose>{pose}</pose>",
            f"{indent}  {geom}",
            *material(self.colour, self.texture, indent + "  "),
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


def include(uri: str, name: str, x: float, y: float, z: float, yaw: float) -> str:
    """A model from models/, placed. Resolved through GZ_SIM_RESOURCE_PATH."""
    return f"""    <include>
      <uri>model://{uri}</uri>
      <name>{name}</name>
      <pose>{x:.4g} {y:.4g} {z:.4g} 0 0 {yaw:.4g}</pose>
    </include>"""


def drone() -> str:
    """The aircraft, part of the world rather than spawned into it once it is running.

    Spawning it afterwards with ros_gz_sim create works for physics and sensors but
    races the GUI: the GUI builds its scene from one snapshot taken at startup, and
    a model created after that snapshot only reaches it through the periodic state
    message, which a loaded GUI drops. The drone then flies invisibly.
    """
    return include(DRONE, DRONE, *DRONE_START)


def wearer() -> str:
    """The glasses' camera, in the world from the start for the same reason as the drone."""
    return include(WEARER, WEARER, *WEARER_START)


def targets(spots) -> tuple[list[str], list[dict]]:
    """The people models and where they really are, one per (x, y, yaw, kind) spot."""
    models, truth = [], []
    for i, (x, y, yaw, kind) in enumerate(spots):
        name = f"person_{i}"
        models.append(include(PEOPLE[i % len(PEOPLE)], name, x, y, 0.0, yaw))
        truth.append({"name": name, "label": "person", "kind": kind,
                      "xyz": [x, y, PERSON_SIZE[2] / 2], "size": list(PERSON_SIZE)})
    return models, truth


def ground(colour: tuple[float, float, float], texture: str | None = None) -> str:
    body = "\n".join(material(colour, texture, " " * 10))
    return f"""    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
{body}
        </visual>
      </link>
    </model>"""


def world(name: str, generator: str, seed: int, scene: str, lights: str, models: str) -> str:
    lat, lon, alt = ORIGIN_LATLON
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/{generator} (seed {seed}). Do not edit by hand. -->
<sdf version="1.9">
  <world name="{name}">

    <physics name="2ms" type="ignore">
      <!-- Lock-stepped with SITL, so each step is also a flight-controller tick.
           1 ms held the full stack to RTF 0.7; config/sitl.parm drops the loop
           to 250 Hz so the 500 Hz gyro clears its 1.8x pre-arm check. -->
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <!-- DART's default FCL checker alone holds a static world to RTF 0.5. -->
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

{scene}

{lights}

{models}

{drone()}

{wearer()}

  </world>
</sdf>
"""


def default_world() -> str:
    """The world the sim was started with, from the WORLD environment variable."""
    return os.environ.get("WORLD", "warehouse")


def targets_path(world: str) -> Path:
    """Where the truth file of a world sits, for the detector stand-in and the scorer."""
    return WORLDS / f"{world}_targets.json"


def write(output: Path, text: str, spots) -> None:
    """The world plus the truth file the detector stand-in and score_search.py read."""
    output.write_text(text)
    truth = output.with_name(f"{output.stem}_targets.json")
    _, entries = targets(spots)
    truth.write_text(json.dumps({"world": output.stem, "targets": entries}, indent=2))
    print(f"wrote {output} and {truth}")
