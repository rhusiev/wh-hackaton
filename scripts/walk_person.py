#!/usr/bin/env python3
"""Walk a person in the running sim to a new spot, to test tracking people who move.

    ./run.sh walk person_1 4.5 7.0              # at 1 m/s
    ./run.sh walk person_1 4.5 7.0 --speed 0.5

The people are static models, so this moves them in small steps through
Gazebo's set_pose service. Where each one ends up goes to /tmp/moved_people.json
for score_search.py. It does not avoid racks: pick a straight, clear line.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time

from score_search import MOVED
from worldgen import default_world, targets_path


def set_pose(world: str, name: str, x: float, y: float, yaw: float) -> None:
    request = (f'name: "{name}", position: {{x: {x}, y: {y}, z: 0}}, '
               f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
    subprocess.run(["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", request],
                   check=True, capture_output=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--speed", type=float, default=1.0, help="m/s")
    parser.add_argument("--world", default=default_world())
    args = parser.parse_args()

    moved = json.loads(MOVED.read_text()) if MOVED.exists() else {}
    start = moved.get(args.name) or next(
        t["xyz"][:2] for t in json.loads(targets_path(args.world).read_text())["targets"]
        if t["name"] == args.name)
    distance = math.dist(start, (args.x, args.y))
    heading = math.atan2(args.y - start[1], args.x - start[0])
    # Each service call takes a good part of a second, so progress goes by the clock.
    began = time.monotonic()
    done = 0.0
    while done < 1.0:
        done = min(1.0, args.speed * (time.monotonic() - began) / distance) if distance else 1.0
        set_pose(args.world, args.name, start[0] + (args.x - start[0]) * done,
                 start[1] + (args.y - start[1]) * done, heading)
    moved[args.name] = [args.x, args.y]
    MOVED.write_text(json.dumps(moved))
    print(f"{args.name} walked {distance:.1f} m to {args.x}, {args.y}")


if __name__ == "__main__":
    main()
