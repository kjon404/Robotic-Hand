# angles.py  — adaptive 0..1 mapping, smooth full range
from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from typing import List, Iterable

ORDER = ["thumb", "index", "middle", "ring", "pinky"]
WRIST = 0

# Angle triplets: anchor at wrist for the first joint of each finger
ANGLE_TRIPLETS = {
    "thumb":  [(WRIST, 1, 2), (1, 2, 3), (2, 3, 4)],      # CMC, MCP, IP
    "index":  [(WRIST, 5, 6), (5, 6, 7), (6, 7, 8)],      # MCP, PIP, DIP
    "middle": [(WRIST, 9,10), (9,10,11), (10,11,12)],
    "ring":   [(WRIST,13,14), (13,14,15), (14,15,16)],
    "pinky":  [(WRIST,17,18), (17,18,19), (18,19,20)],
}

# PIP dominates curl perceptually
W_THUMB = np.array([1.0, 0.30, 1.50], dtype=np.float32)       # favor MCP for thumb
W_FING  = np.array([1.00, 1.50, 0.30], dtype=np.float32)       # MCP,PIP,DIP

def _unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, eps)

def _angle_at_b(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ab = _unit(a - b); cb = _unit(c - b)
    cosang = float(np.clip(np.dot(ab, cb), -1.0, 1.0))
    return float(np.arccos(cosang))

def _joint_angles_15(xyz21: np.ndarray) -> List[float]:
    P = np.asarray(xyz21, dtype=np.float32)
    out: List[float] = []
    for name in ORDER:
        for i, j, k in ANGLE_TRIPLETS[name]:
            out.append(_angle_at_b(P[i], P[j], P[k]))
    return out  # radians, length 15

def _bends_from_angles(angles_rad: Iterable[float]) -> np.ndarray:
    a = np.asarray(list(angles_rad), dtype=np.float32)
    bends = np.maximum(0.0, np.pi - a)      # bend magnitude (radians)
    return bends.reshape(5, 3)               # (finger, joint)

@dataclass
class AdaptiveMinMax:
    """Adaptive per-finger min/max with a small margin to avoid snapping."""
    alpha: float = 0.02
    margin: float = 0.02
    low: np.ndarray = None
    high: np.ndarray = None
    inited: bool = False

    def step(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)  # shape (5,)
        if not self.inited:
            self.low = x.copy()
            self.high = x.copy()
            self.inited = True
        # move low down and high up slowly as new extremes appear
        self.low  += self.alpha * (np.minimum(self.low,  x) - self.low)
        self.high += self.alpha * (np.maximum(self.high, x) - self.high)
        # ensure ordering
        self.high = np.maximum(self.high, self.low + 1e-5)

        span = self.high - self.low
        lo = self.low  + self.margin * span
        hi = self.high - self.margin * span
        span = np.maximum(hi - lo, 1e-5)
        y = (x - lo) / span
        return np.clip(y, 0.0, 1.0)

class HandCurls:
    """
    OOP wrapper that yields five curls in [0..1] and adapts so you always use
    the full range between 0 and 1 over time. No calibration needed.
    """
    def __init__(self, alpha: float = 0.02, margin: float = 0.02) -> None:
        self.adapt = AdaptiveMinMax(alpha=alpha, margin=margin)

    def angles15(self, xyz21: np.ndarray) -> List[float]:
        return _joint_angles_15(xyz21)

    def bends53(self, xyz21: np.ndarray) -> np.ndarray:
        return _bends_from_angles(self.angles15(xyz21))

    def curls5(self, xyz21: np.ndarray) -> np.ndarray:
        bends = self.bends53(xyz21)               # (5,3) bends in radians
        curls_raw = np.empty(5, dtype=np.float32)

        for idx, name in enumerate(ORDER):
            vals = bends[idx]                     # three joints
            if name == "thumb":
                curls_raw[idx] = float(np.dot(vals, W_THUMB))
            else:
                curls_raw[idx] = float(np.dot(vals, W_FING))

        # map raw per-finger bend to adaptive 0..1
        curls = self.adapt.step(curls_raw)
        return curls

    # Provided for completeness if you map to servos here
    def to_servo_angles(
        self,
        curls: np.ndarray,
        mins: Iterable[int],
        maxs: Iterable[int],
        closed_is_1: bool = True,
    ) -> np.ndarray:
        c = np.asarray(curls, dtype=np.float32)
        if not closed_is_1:
            c = 1.0 - c
        mins = np.asarray(list(mins), dtype=np.float32)
        maxs = np.asarray(list(maxs), dtype=np.float32)
        return (mins + c * (maxs - mins)).astype(np.int16)

