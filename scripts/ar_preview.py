#!/usr/bin/env python3
"""Draw what the AR glasses would show, from nothing but the AR WebSocket feed.

Left is the minimap: the grid, the drone and every tracked person with their head.
A person no longer where they were last seen is drawn pale, with how long ago.
Right is the wallhack view of the wearer, who starts at --viewer and can be walked
around: each person's box and head box projected into their sight, through walls,
with the distance. The room is drawn as a wireframe traced from the same grid,
standing in for what the wearer's own eyes would supply - real glasses are
see-through and would send only the overlay.

    ./run.sh preview                          # live window
    ./run.sh preview --save ar.png            # one frame to a file
    ./run.sh preview --viewer -15 0 0         # x, y and yaw the wearer starts at

W and S walk, A and D turn, q or Escape quits. The keys are read between frames
of the feed, so holding one moves at the feed's rate rather than the keyboard's.
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
STEP = 0.5                  # m per keypress, about a stride
TURN = math.radians(10)
NEAR = 0.3                  # m, nothing closer than this can be projected
WALL_H = 3.0                # m, how tall to draw a mapped obstacle: the grid is a
                            # 2.3 m slice and has no heights in it
WALL_SIMPLIFY = 1.5         # cells of detour a wireframe corner may cut
GRID_COLOURS = np.array([[235, 235, 235], [40, 40, 40], [150, 150, 150]], dtype=np.uint8)
OCCUPIED = 1                # the index into GRID_COLOURS the mapper marks obstacles with
PERSON, HEAD, DRONE, WEARER = (0, 90, 255), (0, 220, 255), (200, 80, 0), (60, 160, 60)
LOST = (140, 140, 200)
WALL = (90, 90, 90)


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


def outlines(grid: dict) -> list[np.ndarray]:
    """The mapped obstacles as closed polygons in map coordinates.

    The grid is a top-down picture, so the edge of an occupied region is exactly
    where a wall face is. Tracing it costs far fewer lines than drawing a box per
    cell, and simplifying it keeps a straight wall one straight line.
    """
    cells = np.frombuffer(base64.b64decode(grid["cells"]), np.uint8).reshape(grid["h"], grid["w"])
    found, _ = cv2.findContours((cells == OCCUPIED).astype(np.uint8),
                                cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    polygons = []
    for contour in found:
        corners = cv2.approxPolyDP(contour, WALL_SIMPLIFY, True).reshape(-1, 2)
        if len(corners) >= 2:
            polygons.append((corners + 0.5) * grid["res"] + [grid["x0"], grid["y0"]])
    return polygons


def wallhack(frame: dict, viewer: tuple[float, float, float]) -> np.ndarray:
    image = np.full((VIEW_H, VIEW_W, 3), 30, np.uint8)
    vx, vy, yaw = viewer
    focal = VIEW_W / 2 / math.tan(VIEW_HFOV / 2)
    forward, left = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])

    def project(x: float, y: float, z: float) -> tuple[float, float, float] | None:
        rel = np.array([x - vx, y - vy])
        depth = float(rel @ forward)
        if depth < NEAR:
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

    def edge(a: np.ndarray, b: np.ndarray, za: float, zb: float) -> None:
        """A line between two map points, cut where it would pass behind the wearer.

        Depth comes from x and y alone, so the crossing is found in the plan and the
        height is carried along it.
        """
        da, db = float((a - (vx, vy)) @ forward), float((b - (vx, vy)) @ forward)
        if da < NEAR and db < NEAR:
            return
        if min(da, db) < NEAR:
            t = (NEAR - da) / (db - da)
            if da < NEAR:
                a, za = a + t * (b - a), za + t * (zb - za)
            else:
                b, zb = a + t * (b - a), za + t * (zb - za)
        start, end = project(*a, za), project(*b, zb)
        if start and end:
            cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), WALL, 1)

    if frame.get("map"):
        for polygon in outlines(frame["map"]):
            for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                edge(a, b, 0.0, 0.0)
                edge(a, b, WALL_H, WALL_H)
                edge(a, a, 0.0, WALL_H)

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


def walk(viewer: tuple[float, float, float], key: int) -> tuple[float, float, float]:
    """W and S step along the way the wearer faces, A and D turn on the spot."""
    x, y, yaw = viewer
    match chr(key & 0xFF).lower():
        case "w":
            return x + STEP * math.cos(yaw), y + STEP * math.sin(yaw), yaw
        case "s":
            return x - STEP * math.cos(yaw), y - STEP * math.sin(yaw), yaw
        case "a":
            return x, y, yaw + TURN
        case "d":
            return x, y, yaw - TURN
    return viewer


async def show(url: str, viewer: tuple[float, float, float], save: str | None) -> None:
    async with websockets.connect(url) as socket:
        while True:
            image = compose(json.loads(await socket.recv()), viewer)
            if save:
                cv2.imwrite(save, image)
                print(f"wrote {save}")
                return
            cv2.imshow("AR preview", image)
            key = cv2.waitKey(1)
            if key in (ord("q"), 27):
                return
            viewer = walk(viewer, key)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="ws://localhost:8790")
    parser.add_argument("--viewer", type=float, nargs=3, default=(-15.5, 0.0, 0.0),
                        metavar=("X", "Y", "YAW"), help="where the wearer starts, map frame")
    parser.add_argument("--save", help="write one frame to this path and exit")
    args = parser.parse_args()
    try:
        asyncio.run(show(args.url, tuple(args.viewer), args.save))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
