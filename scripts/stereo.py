"""OAK-D stereo depth model shared by the sim's depth noise, detector and demo."""

from __future__ import annotations

import numpy as np

DEPTH_MIN = 0.7        # stereo minimum at 800p
DEPTH_RELIABLE = 12.0  # avoidance trusts nothing farther
DEPTH_MAX = 30.0       # still usable for search, with much larger error

BASELINE = 0.075       # m
FOCAL_PX = 466.1       # colour-aligned, 640 px across 68.8 deg
SUBPIXEL = 0.125       # px of disparity resolution


def sigma(z):
    """1-sigma depth error. Depth is b*f/disparity, so a fixed disparity error grows as z²."""
    return 0.005 + z * z * SUBPIXEL / (BASELINE * FOCAL_PX)


def dropout(z):
    """Chance that a patch returns no depth. Zero inside the reliable range, 50 % at the max."""
    return 0.5 * np.clip((z - DEPTH_RELIABLE) / (DEPTH_MAX - DEPTH_RELIABLE), 0.0, 1.0)
