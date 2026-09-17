#!/usr/bin/env python3
"""Take off and explore the warehouse with a swappable strategy.

    ./scripts/explore.py                        # lawnmower sweep of the known aisles
    ./scripts/explore.py --strategy frontier    # no prior layout, fly to the unknown in /map
    ./scripts/explore.py --strategy watch       # frontier, then keep everyone found in view
    ./scripts/explore.py --strategy my_search.py:MySearch

A strategy is a class built from the parsed arguments, with run(flight) flying
the mission through flight.Flight: here, heading, grid, people, fly_to and turn. It may
add its own arguments with a static add_arguments(parser).
"""

from __future__ import annotations

import argparse

import rclpy
from copter import run
from flight import Flight
from plugin import load

STRATEGIES = {"sweep": "sweep:Sweep", "frontier": "frontier:FrontierExplorer", "watch": "watch:Watch"}


class Explore(Flight):
    def __init__(self, altitude: float, strategy) -> None:
        super().__init__(altitude)
        self.strategy = strategy

    def run(self) -> None:
        self.take_off()
        self.strategy.run(self)


def main() -> None:
    first = argparse.ArgumentParser(add_help=False)
    first.add_argument("--strategy", default="sweep",
                       help=f"{', '.join(STRATEGIES)}, or module:Class / path.py:Class")
    known, _ = first.parse_known_args()
    strategy = load(known.strategy, STRATEGIES)

    parser = argparse.ArgumentParser(description=__doc__, parents=[first],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    # Above the 2.45 m shelf deck, not level with it: seen edge-on, a deck is missing from the map.
    parser.add_argument("--altitude", type=float, default=2.8)
    if hasattr(strategy, "add_arguments"):
        strategy.add_arguments(parser)
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    run(Explore(parsed.altitude, strategy(parsed)))


if __name__ == "__main__":
    main()
