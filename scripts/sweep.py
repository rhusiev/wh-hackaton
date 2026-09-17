"""Lawnmower sweep of the warehouse's five known aisles: the one strategy tied to that world."""

from __future__ import annotations

import argparse
import math

from flight import Flight

# Rack rows sit at y = +/-7.5 and +/-2.5 and are 1.1 m deep, so these are the
# five clear lanes. x stops short of the 32 m hall's end walls.
LANES = (-9.3, -5.0, 0.0, 5.0, 9.3)
X_SPAN = (-13.0, 13.0)


class Sweep:
    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--lanes", type=int, default=len(LANES),
                            help="how many of the five lanes to sweep")

    def __init__(self, args: argparse.Namespace) -> None:
        self.waypoints = []
        for i, y in enumerate(LANES[:args.lanes]):
            x0, x1 = X_SPAN if i % 2 == 0 else X_SPAN[::-1]
            self.waypoints += [(x0, y), (x1, y)]

    def run(self, flight: Flight) -> None:
        log = flight.get_logger()
        previous = None
        for x, y in self.waypoints:
            yaw = 0.0 if previous is None else math.atan2(y - previous[1], x - previous[0])
            log.info(f"leg to map ({x:.1f}, {y:.1f})")
            if not flight.fly_to(x, y, yaw):
                log.error("leg timed out, stopping the sweep")
                return
            previous = (x, y)
        log.info("sweep complete, holding at the last waypoint")
