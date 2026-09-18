#!/usr/bin/env python3
"""Draw what the AR glasses would show, from nothing but the AR WebSocket feed.

One window: what the wearer sees, with the minimap inset in its corner. Each
tracked person gets a box and a head box projected into their sight, through
walls, with the distance; a person no longer where they were last seen is drawn
pale, with how long ago. Behind the overlay is either a real camera image
(--camera) or a wireframe of the room traced from the mapped grid, which stands
in for what see-through glasses would let the eye supply by itself.

    ./run.sh preview                          # live window
    ./run.sh preview --camera 0               # overlay on a real camera
    ./run.sh preview --save ar.png            # one frame to a file
    ./run.sh preview --viewer -15 0 0         # x, y and yaw the wearer starts at

The window is resizable and scales its contents to fit.

    W S     walk forward and back        arrows or I J K L    look around
    A D     step left and right          q or Escape          quit

Looking left and right turns the wearer; looking up and down pitches the view
without leaving the floor, the way a head does.

The camera and the feed run at different rates, so they are read separately: the
newest payload is kept by a background task and drawn onto whatever camera frame
is current. The picture therefore never waits for the feed, and the boxes are
always the most recent ones rather than a frame-matched pair - matching them
would need a timestamp the camera does not share with the drone.
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
PITCH_LIMIT = math.radians(80)  # past this the horizon is behind you and the view tumbles
NEAR = 0.3                  # m, nothing closer than this can be projected
WALL_H = 3.0                # m, how tall to draw a mapped obstacle: the grid is a
                            # 2.3 m slice and has no heights in it
WALL_SIMPLIFY = 1.5         # cells of detour a wireframe corner may cut
GRID_COLOURS = np.array([[235, 235, 235], [40, 40, 40], [150, 150, 150]], dtype=np.uint8)
OCCUPIED = 1                # the index into GRID_COLOURS the mapper marks obstacles with
PERSON, HEAD, DRONE, WEARER = (0, 90, 255), (0, 220, 255), (200, 80, 0), (60, 160, 60)
LOST = (140, 140, 200)
WALL, FLOOR, HORIZON = (90, 90, 90), (55, 55, 55), (45, 45, 45)
FLOOR_STEP, FLOOR_EXTENT = 2.0, 24.0    # m between floor lines, and how far they run
FADE_FULL, FADE_MIN = 30.0, 0.3         # a line is dimmest this far off, but never darker
WINDOW = "AR glasses"
INSET_W = 220               # px wide the minimap is drawn in the corner
INSET_MARGIN = 10
DRAW_RATE = 30.0            # Hz the window is redrawn at when there is no camera to pace it

# Where the wearer is and where they are looking: x, y, yaw, pitch.
Viewer = tuple[float, float, float, float]


def colour(target: dict) -> tuple[int, int, int]:
    return LOST if target["status"] == "lost" else PERSON


def minimap(frame: dict, viewer: Viewer) -> np.ndarray:
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
    arrow(viewer[0], viewer[1], viewer[2], WEARER)
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


def wallhack(frame: dict, viewer: Viewer, scene: np.ndarray | None = None) -> np.ndarray:
    """The overlay, over a real camera image if one is given and a wireframe if not.

    With a camera the room is already in the picture, so the floor, horizon and
    traced walls are left out: they would sit at the wearer's assumed pose rather
    than the camera's real one, and disagree with what is behind them.
    """
    image = cv2.resize(scene, (VIEW_W, VIEW_H)) if scene is not None \
        else np.full((VIEW_H, VIEW_W, 3), 30, np.uint8)
    vx, vy, yaw, pitch = viewer
    eye = np.array([vx, vy, EYE_HEIGHT])
    focal = VIEW_W / 2 / math.tan(VIEW_HFOV / 2)
    # The three axes of the head: where it looks, its left, and its up. Pitch tilts
    # the first and the third and leaves left alone, which is what stops a look
    # upwards from rolling the horizon.
    forward = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
                        math.sin(pitch)])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.array([-math.sin(pitch) * math.cos(yaw), -math.sin(pitch) * math.sin(yaw),
                   math.cos(pitch)])

    def project(x: float, y: float, z: float) -> tuple[float, float, float] | None:
        rel = np.array([x, y, z]) - eye
        depth = float(rel @ forward)
        if depth < NEAR:
            return None
        return (VIEW_W / 2 - focal * float(rel @ left) / depth,
                VIEW_H / 2 - focal * float(rel @ up) / depth, depth)

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

    def edge(a: np.ndarray, b: np.ndarray, za: float, zb: float, base=WALL) -> None:
        """A line between two map points, cut where it would pass behind the wearer.

        Depth comes from x and y alone, so the crossing is found in the plan and the
        height is carried along it. The line dims with distance, which is the only
        depth cue a wireframe has.
        """
        pa, pb = np.array([*a, za]), np.array([*b, zb])
        da, db = float((pa - eye) @ forward), float((pb - eye) @ forward)
        if da < NEAR and db < NEAR:
            return
        if min(da, db) < NEAR:
            t = (NEAR - da) / (db - da)
            if da < NEAR:
                a, za = a + t * (b - a), za + t * (zb - za)
            else:
                b, zb = a + t * (b - a), za + t * (zb - za)
        start, end = project(*a, za), project(*b, zb)
        if not (start and end):
            return
        shade = max(FADE_MIN, 1.0 - (da + db) / 2 / FADE_FULL)
        cv2.line(image, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])),
                 tuple(c * shade for c in base), 1)

    if scene is None:
        # The horizon is where a ray at eye height ends up, which pitch moves.
        horizon = int(VIEW_H / 2 + focal * math.tan(pitch))
        if 0 <= horizon < VIEW_H:
            cv2.line(image, (0, horizon), (VIEW_W, horizon), HORIZON, 1)
        # Lines on whole multiples of the step, so the floor stays put as the wearer walks.
        near_x, near_y = (math.floor(c / FLOOR_STEP) * FLOOR_STEP for c in (vx, vy))
        for offset in np.arange(-FLOOR_EXTENT, FLOOR_EXTENT + FLOOR_STEP, FLOOR_STEP):
            x, y = near_x + offset, near_y + offset
            edge(np.array([near_x - FLOOR_EXTENT, y]), np.array([near_x + FLOOR_EXTENT, y]),
                 0.0, 0.0, FLOOR)
            edge(np.array([x, near_y - FLOOR_EXTENT]), np.array([x, near_y + FLOOR_EXTENT]),
                 0.0, 0.0, FLOOR)

        if frame.get("map"):
            for polygon in outlines(frame["map"]):
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    edge(a, b, 0.0, 0.0)
                    edge(a, b, WALL_H, WALL_H)
                    edge(a, a, 0.0, WALL_H)

    if frame.get("drone"):
        d = frame["drone"]
        if box(d["x"], d["y"], d["z"], 0.6, 0.3, DRONE) is not None:
            u, v, _ = project(d["x"], d["y"], d["z"])
            cv2.putText(image, "drone", (int(u) - 18, int(v) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, DRONE, 1)

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


def compose(frame: dict, viewer: Viewer, scene: np.ndarray | None = None) -> np.ndarray:
    """The view with the minimap inset in its top right, which is the whole picture."""
    image = wallhack(frame, viewer, scene)
    plan = minimap(frame, viewer)
    inset = cv2.resize(plan, (INSET_W, round(INSET_W * plan.shape[0] / plan.shape[1])))
    h, w = inset.shape[:2]
    if h + 2 * INSET_MARGIN > VIEW_H:            # a tall map would cover the view
        inset = cv2.resize(inset, (round(w * (VIEW_H - 2 * INSET_MARGIN) / h),
                                   VIEW_H - 2 * INSET_MARGIN))
        h, w = inset.shape[:2]
    x = VIEW_W - w - INSET_MARGIN
    image[INSET_MARGIN:INSET_MARGIN + h, x:x + w] = inset
    cv2.rectangle(image, (x - 1, INSET_MARGIN - 1), (x + w, INSET_MARGIN + h),
                  (255, 255, 255), 1)
    return image


# GTK reports the arrows in the 65361-65364 block and the other backends in
# 81-84, so both are mapped onto the letters that do the same thing everywhere.
ARROWS = {65361: "j", 65362: "i", 65363: "l", 65364: "k",
          81: "j", 82: "i", 83: "l", 84: "k"}


def walk(viewer: Viewer, key: int) -> Viewer:
    """WASD moves the wearer over the floor, IJKL and the arrows aim their head."""
    x, y, yaw, pitch = viewer
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-math.sin(yaw), math.cos(yaw))
    match ARROWS.get(key, chr(key & 0xFF).lower()):
        case "w":
            return x + STEP * forward[0], y + STEP * forward[1], yaw, pitch
        case "s":
            return x - STEP * forward[0], y - STEP * forward[1], yaw, pitch
        case "a":
            return x + STEP * left[0], y + STEP * left[1], yaw, pitch
        case "d":
            return x - STEP * left[0], y - STEP * left[1], yaw, pitch
        case "j":
            return x, y, yaw + TURN, pitch
        case "l":
            return x, y, yaw - TURN, pitch
        case "i":
            return x, y, yaw, min(pitch + TURN, PITCH_LIMIT)
        case "k":
            return x, y, yaw, max(pitch - TURN, -PITCH_LIMIT)
    return viewer


def open_camera(source: str | None) -> cv2.VideoCapture | None:
    """The wearer's own camera, by index or device path. Absent is not an error."""
    if source is None:
        return None
    capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not capture.isOpened():
        # Not every /dev/videoN captures: a machine's first node is often a virtual
        # camera or the metadata half of a real one, and neither yields a frame.
        raise SystemExit(
            f"cannot open camera {source}. In the container it is whatever "
            "CAMERA=/dev/videoN ./run.sh up passed in; check on the host which node "
            "captures with: v4l2-ctl --list-devices")
    return capture


async def feed(url: str, latest: dict) -> None:
    """Keep the newest payload in latest["frame"], so drawing never waits on it."""
    async with websockets.connect(url) as socket:
        while True:
            latest["frame"] = json.loads(await socket.recv())


async def show(url: str, viewer: Viewer, save: str | None,
               camera: cv2.VideoCapture | None) -> None:
    latest: dict = {}
    reader = asyncio.create_task(feed(url, latest))
    try:
        while "frame" not in latest:
            await asyncio.sleep(0.01)
            if reader.done():
                await reader                     # the connection failed; report why
        if save:
            read, scene = camera.read() if camera else (False, None)
            cv2.imwrite(save, compose(latest["frame"], viewer, scene if read else None))
            print(f"wrote {save}")
            return

        # WINDOW_NORMAL lets the window be dragged to any size and scales the frame
        # into it; KEEPRATIO stops that scaling from stretching the view.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        while True:
            scene = None
            if camera is not None:
                read, scene = camera.read()
                if not read:
                    scene = None
            cv2.imshow(WINDOW, compose(latest["frame"], viewer, scene))
            key = cv2.waitKey(1)
            if key in (ord("q"), 27):
                return
            viewer = walk(viewer, key)
            # A camera read blocks until its next frame and paces the loop by itself;
            # without one, nothing would and the loop would spin on a core.
            await asyncio.sleep(0 if camera is not None else 1.0 / DRAW_RATE)
    finally:
        reader.cancel()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="ws://localhost:8790")
    parser.add_argument("--viewer", type=float, nargs=3, default=(-15.5, 0.0, 0.0),
                        metavar=("X", "Y", "YAW"), help="where the wearer starts, map frame")
    parser.add_argument("--hfov", type=float, help="camera's horizontal field of view, degrees")
    parser.add_argument("--save", help="write one frame to this path and exit")
    parser.add_argument("--camera", help="index or device of the camera to draw the overlay on")
    args = parser.parse_args()
    if args.hfov:
        global VIEW_HFOV
        VIEW_HFOV = math.radians(args.hfov)
    camera = open_camera(args.camera)
    try:
        asyncio.run(show(args.url, (*args.viewer, 0.0), args.save, camera))
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.release()


if __name__ == "__main__":
    main()
