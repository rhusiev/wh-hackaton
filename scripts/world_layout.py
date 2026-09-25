#!/usr/bin/env python3
"""Every box of a generated world, for the glasses to draw the sim around the wearer.

Writes web/worlds/<world>.json: {"world": name, "boxes": [[x, y, z, sx, sy, sz, yaw, r, g, b], ...],
"people": [[mesh, x, y, yaw], ...], "wearer": [x, y, yaw_deg], "drone": [[shape, a, b, c, r, g, b, *pose], ...]}.
Everything is in the world frame except the drone's shapes, which are in its body frame: a box's size or a
cylinder's radius, radius and length, then the top three rows of the shape's pose matrix. The boxes are the
static ones worldgen.py writes, the people the meshes it includes from models/people; where the drone is and
what it found come from the feed.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from worldgen import DRONE, PEOPLE, WEARER_START, WORLDS

LAYOUTS = WORLDS.parent / "web" / "worlds"
MODELS = WORLDS.parent / "models"


def numbers(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def pose(text: str | None) -> np.ndarray:
    x, y, z, roll, pitch, yaw = numbers(text or "0 0 0 0 0 0")
    c, s = np.cos([roll, pitch, yaw]), np.sin([roll, pitch, yaw])
    m = np.eye(4)
    m[:3, :3] = (np.array([[c[2], -s[2], 0], [s[2], c[2], 0], [0, 0, 1]])
                 @ np.array([[c[1], 0, s[1]], [0, 1, 0], [-s[1], 0, c[1]]])
                 @ np.array([[1, 0, 0], [0, c[0], -s[0]], [0, s[0], c[0]]]))
    m[:3, 3] = x, y, z
    return m


def boxes(sdf: str) -> list[list[float]]:
    out = []
    for model in ET.fromstring(sdf).iter("model"):
        if model.findtext("static") != "true":
            continue
        placed = pose(model.findtext("pose"))
        for visual in model.iter("visual"):
            size = visual.find("geometry/box/size")
            if size is None:
                continue
            m = placed @ pose(visual.findtext("pose"))
            r, g, b, _ = numbers(visual.findtext("material/diffuse", "0.7 0.7 0.7 1"))
            out.append([round(float(v), 3) for v in (
                *m[:3, 3], *numbers(size.text), math.atan2(m[1, 0], m[0, 0]), r, g, b)])
    return out


def people(sdf: str) -> list[list]:
    out = []
    for include in ET.fromstring(sdf).iter("include"):
        model = include.findtext("uri").removeprefix("model://")
        if model in PEOPLE:
            x, y, *_, yaw = numbers(include.findtext("pose"))
            mesh = ET.parse(MODELS / "people" / model / "model.sdf").findtext(".//visual//mesh/uri")
            out.append([mesh.removeprefix("model://"), x, y, yaw])
    return out


def drone() -> list[list]:
    out = []
    for link in ET.parse(MODELS / DRONE / "model.sdf").iter("link"):
        for visual in link.iter("visual"):
            if (box := visual.find("geometry/box/size")) is not None:
                shape, size = "box", numbers(box.text)
            elif (cylinder := visual.find("geometry/cylinder")) is not None:
                radius = float(cylinder.findtext("radius"))
                shape, size = "cylinder", [radius, radius, float(cylinder.findtext("length"))]
            else:
                continue
            r, g, b, _ = numbers(visual.findtext("material/diffuse", "0.7 0.7 0.7 1"))
            m = pose(link.findtext("pose")) @ pose(visual.findtext("pose"))
            out.append([shape, *size, r, g, b, *np.round(m[:3].ravel(), 4).tolist()])
    return out


def write_layout(world: str, sdf: str) -> Path:
    LAYOUTS.mkdir(exist_ok=True)
    path = LAYOUTS / f"{world}.json"
    x, y, _, yaw = WEARER_START
    path.write_text(json.dumps({"world": world, "boxes": boxes(sdf), "people": people(sdf),
                                "wearer": [x, y, round(math.degrees(yaw), 1)], "drone": drone()}, separators=(",", ":")))
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("worlds", nargs="*", default=["warehouse", "garden"])
    for world in parser.parse_args().worlds:
        print(f"wrote {write_layout(world, (WORLDS / f'{world}.sdf').read_text())}")


if __name__ == "__main__":
    main()
