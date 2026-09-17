"""Person detection networks: what finds people and their heads in a colour image.

A network returns a Person per detection: a box and, when it can tell, a head box.
The detector node only depends on that, so any class with the same call works.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import cv2
import numpy as np
import onnxruntime as ort

FACE = slice(0, 5)  # nose, eyes, ears
SHOULDERS = [5, 6]
PAD = 114 / 255
NMS_IOU = 0.5


@dataclass
class Person:
    box: np.ndarray  # x0, y0, x1, y1 in image pixels
    score: float
    head: np.ndarray | None


def head_box(keypoints: np.ndarray, box: np.ndarray, min_conf: float) -> np.ndarray | None:
    face = keypoints[FACE]
    face = face[face[:, 2] > min_conf, :2]
    shoulders = keypoints[SHOULDERS]
    has_shoulders = bool((shoulders[:, 2] > min_conf).all())

    # A head is about 1/8 of a standing body and half the shoulder width. Face
    # keypoints are the better anchor, but from behind only the shoulders remain.
    size = (box[3] - box[1]) / 8
    if has_shoulders:
        size = max(size, 0.55 * abs(shoulders[0, 0] - shoulders[1, 0]))
    if len(face) >= 2:
        size = max(size, 1.6 * np.ptp(face, axis=0).max())
        cx, cy = face.mean(axis=0)
        cy -= 0.15 * size  # eyes and ears sit below the middle of the head
    elif has_shoulders:
        cx, cy = shoulders[:, :2].mean(axis=0)
        cy -= size
    else:
        return None
    return np.array([cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2])


class PersonNetwork(Protocol):
    """Anything that finds people in an RGB image; see person_detector.py for how it is picked."""

    def __call__(self, rgb: np.ndarray) -> list[Person]: ...


class PoseNetwork:
    """YOLO11-pose exported to ONNX, run with onnxruntime on the CPU."""

    def __init__(self, model: str, threads: int, min_score: float, min_keypoint: float) -> None:
        self.min_score, self.min_keypoint = min_score, min_keypoint
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        self.session = ort.InferenceSession(model, options, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        self.height, self.width = self.input.shape[2:]

    def __call__(self, rgb: np.ndarray) -> list[Person]:
        min_score, min_keypoint = self.min_score, self.min_keypoint
        scale = min(self.width / rgb.shape[1], self.height / rgb.shape[0])
        resized = cv2.resize(rgb, (round(rgb.shape[1] * scale), round(rgb.shape[0] * scale)))
        batch = np.full((1, 3, self.height, self.width), PAD, dtype=np.float32)
        batch[0, :, :resized.shape[0], :resized.shape[1]] = resized.transpose(2, 0, 1) / 255

        # One row per anchor: centre x, y, w, h, score, then x, y, confidence per keypoint.
        rows = self.session.run(None, {self.input.name: batch})[0][0].T
        rows = rows[rows[:, 4] > min_score]
        xywh = np.column_stack([rows[:, :2] - rows[:, 2:4] / 2, rows[:, 2:4]])
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), rows[:, 4].tolist(), min_score, NMS_IOU)

        people = []
        for i in np.ravel(keep):
            box = np.concatenate([xywh[i, :2], xywh[i, :2] + xywh[i, 2:]]) / scale
            keypoints = rows[i, 5:].reshape(17, 3).copy()
            keypoints[:, :2] /= scale
            people.append(Person(box, float(rows[i, 4]), head_box(keypoints, box, min_keypoint)))
        return people
