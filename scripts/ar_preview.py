#!/usr/bin/env python3
"""Draw what the AR glasses would show, from nothing but the AR WebSocket feed.

Left is the minimap: the grid, the drone and every tracked person with their head.
A person no longer where they were last seen is drawn pale, with how long ago.
Right is the wallhack view of someone standing at --viewer: each person's box and
head box projected into their sight, through walls, with the distance.

    ./run.sh preview                          # live window
    ./run.sh preview --save ar.png            # one frame to a file
    ./run.sh preview --viewer -15 0 0         # x, y and yaw of the wearer
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math

import cv2
import numpy as np
import websockets

MINIMAP_SCALE = 16          # px per m
VIEW_W, VIEW_H, VIEW_HFOV = 640, 400, math.radians(60)
EYE_HEIGHT = 1.7
GRID_COLOURS = np.array([[235, 235, 235], [40, 40, 40], [150, 150, 150]], dtype=np.uint8)
PERSON, HEAD, DRONE, WEARER = (0, 90, 255), (0, 220, 255), (200, 80, 0), (60, 160, 60)
LOST = (140, 140, 200)


def colour(target: dict) -> tuple[int, int, int]:
    return LOST if target["status"] == "lost" else PERSON


def minimap(frame: dict, viewer: tuple[float, float, float]) -> np.ndarray:
    grid = frame.get("map")
    if grid is None:
        image = np.full((VIEW_H, VIEW_H), 150, np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        x0 = y0 = -VIEW_H / MINIMAP_SCALE / 2
        rows = VIEW_H
    else:
        cells = np.frombuffer(base64.b64decode(grid["cells"]), np.uint8).reshape(grid["h"], grid["w"])
        cell_px = grid["res"] * MINIMAP_SCALE
        image = cv2.resize(GRID_COLOURS[cells], None, fx=cell_px, fy=cell_px,
                           interpolation=cv2.INTER_NEAREST)
        x0, y0, rows = grid["x0"], grid["y0"], image.shape[0]
    image = np.ascontiguousarray(image[::-1])  # map y points up

    def px(x: float, y: float) -> tuple[int, int]:
        return int((x - x0) * MINIMAP_SCALE), int(rows - (y - y0) * MINIMAP_SCALE)

    def arrow(x: float, y: float, yaw: float, colour) -> None:
        tip = px(x + 0.8 * math.cos(yaw), y + 0.8 * math.sin(yaw))
        cv2.arrowedLine(image, px(x, y), tip, colour, 2, tipLength=0.4)

    if frame.get("drone"):
        d = frame["drone"]
        arrow(d["x"], d["y"], d["yaw"], DRONE)
    arrow(*viewer, WEARER)
    for target in frame["targets"]:
        cv2.circle(image, px(target["x"], target["y"]), 5, colour(target), -1)
        cv2.putText(image, str(target["id"]), px(target["x"] + 0.3, target["y"] + 0.3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour(target), 1)
        if "head" in target:
            cv2.circle(image, px(target["head"]["x"], target["head"]["y"]), 2, HEAD, -1)
    return image


def wallhack(frame: dict, viewer: tuple[float, float, float]) -> np.ndarray:
    image = np.full((VIEW_H, VIEW_W, 3), 30, np.uint8)
    vx, vy, yaw = viewer
    focal = VIEW_W / 2 / math.tan(VIEW_HFOV / 2)
    forward, left = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])

    def project(x: float, y: float, z: float) -> tuple[float, float, float] | None:
        rel = np.array([x - vx, y - vy])
        depth = float(rel @ forward)
        if depth < 0.3:
            return None
        return (VIEW_W / 2 - focal * float(rel @ left) / depth,
                VIEW_H / 2 - focal * (z - EYE_HEIGHT) / depth, depth)

    def box(x: float, y: float, z: float, width: float, height: float, colour) -> float | None:
        """A screen-facing rectangle centred on x, y, z; returns its distance if drawn."""
        centre = project(x, y, z)
        if centre is None:
            return None
        u, v, depth = centre
        half_w, half_h = focal * width / 2 / depth, focal * height / 2 / depth
        cv2.rectangle(image, (int(u - half_w), int(v - half_h)), (int(u + half_w), int(v + half_h)),
                      colour, 2)
        return depth

    labels: list[tuple[int, int, int, int]] = []
    for target in sorted(frame["targets"], key=lambda t: -math.dist((t["x"], t["y"]), (vx, vy))):
        depth = box(target["x"], target["y"], target["z"], 0.5, target["h"], colour(target))
        if depth is None:
            continue
        if "head" in target:
            head = target["head"]
            box(head["x"], head["y"], head["z"], head["size"], head["size"], HEAD)
        u, v, _ = project(target["x"], target["y"], target["z"] + target["h"] / 2)
        text = f"#{target['id']} {math.dist((target['x'], target['y']), (vx, vy)):.1f} m"
        if target["status"] == "lost":
            text += f", {target['age']:.0f} s ago"
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        x, y = int(u) - w // 2, int(v) - 20
        # Move up past every label already drawn that it would overlap.
        while any(x < lx + lw and lx < x + w and y - h - 2 < ly and ly - lh - 2 < y
                  for lx, ly, lw, lh in labels):
            y -= h + 4
        labels.append((x, y, w, h))
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour(target), 1)
    cv2.putText(image, f"{len(frame['targets'])} people", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return image


def compose(frame: dict, viewer: tuple[float, float, float]) -> np.ndarray:
    left, right = minimap(frame, viewer), wallhack(frame, viewer)
    height = max(left.shape[0], right.shape[0])
    pad = lambda img: cv2.copyMakeBorder(img, 0, height - img.shape[0], 0, 0,
                                         cv2.BORDER_CONSTANT, value=(0, 0, 0))
    return np.hstack([pad(left), pad(right)])


async def show(url: str, viewer: tuple[float, float, float], save: str | None) -> None:
    async with websockets.connect(url) as socket:
        while True:
            image = compose(json.loads(await socket.recv()), viewer)
            if save:
                cv2.imwrite(save, image)
                print(f"wrote {save}")
                return
            cv2.imshow("AR preview", image)
            if cv2.waitKey(1) in (ord("q"), 27):
                return


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="ws://localhost:8790")
    parser.add_argument("--viewer", type=float, nargs=3, default=(-15.5, 0.0, 0.0),
                        metavar=("X", "Y", "YAW"), help="where the wearer stands, map frame")
    parser.add_argument("--save", help="write one frame to this path and exit")
    args = parser.parse_args()
    try:
        asyncio.run(show(args.url, tuple(args.viewer), args.save))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
