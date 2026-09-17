#!/usr/bin/env python3
"""Compare the people the AR feed reports with where the world generator put them.

    ./run.sh score            # after or during ./run.sh explore
    ./run.sh score --radius 1

Prints each real person as found or missed with the position error, then the
confirmed targets that match nobody, then the lost ones, which are not scored.
Exits 1 if anyone was missed.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
from pathlib import Path

import websockets

TRUTH = Path(__file__).resolve().parent.parent / "worlds" / "warehouse_targets.json"
MOVED = Path("/tmp/moved_people.json")  # written by walk_person.py


async def snapshot(url: str) -> dict:
    async with websockets.connect(url) as socket:
        return json.loads(await socket.recv())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://localhost:8790")
    parser.add_argument("--radius", type=float, default=1.0,
                        help="how far a report may be from a person to count as them, m")
    args = parser.parse_args()

    people = json.loads(TRUTH.read_text())["targets"]
    moved = json.loads(MOVED.read_text()) if MOVED.exists() else {}
    for person in people:
        if person["name"] in moved:
            person["xyz"][:2] = moved[person["name"]]
    targets = asyncio.run(snapshot(args.url))["targets"]
    reported = [t for t in targets if t["status"] == "confirmed"]
    lost = [t for t in targets if t["status"] == "lost"]

    # Closest pairs first, so one report cannot claim two people.
    pairs = sorted(
        (math.dist(p["xyz"][:2], (r["x"], r["y"])), i, j)
        for i, p in enumerate(people) for j, r in enumerate(reported))
    match: dict[int, tuple[float, int]] = {}
    used = set()
    for distance, i, j in pairs:
        if distance <= args.radius and i not in match and j not in used:
            match[i] = distance, j
            used.add(j)

    for i, person in enumerate(people):
        where = f"{person['name']} ({person['kind']}) at {person['xyz'][0]:.1f}, {person['xyz'][1]:.1f}"
        if i in match:
            distance, j = match[i]
            head = "with head" if "head" in reported[j] else "no head"
            print(f"found   {where}: off by {distance:.2f} m, {reported[j]['hits']} hits, {head}")
        else:
            print(f"missed  {where}")
    for j, target in enumerate(reported):
        if j not in used:
            print(f"false   target {target['id']} at {target['x']:.1f}, {target['y']:.1f}, "
                  f"{target['hits']} hits")

    for target in lost:
        print(f"lost    target {target['id']} last seen at {target['x']:.1f}, {target['y']:.1f} "
              f"{target['age']:.0f} s ago")
    print(f"{len(match)}/{len(people)} people found, {len(reported) - len(used)} false targets, "
          f"{len(lost)} lost")
    sys.exit(0 if len(match) == len(people) else 1)


if __name__ == "__main__":
    main()
