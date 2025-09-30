# angles.py
import numpy as np

# MediaPipe hand landmark finger indices
FINGERS = {
    "thumb":  [1, 2, 3, 4],   # CMC, MCP, IP, TIP
    "index":  [5, 6, 7, 8],   # MCP, PIP, DIP, TIP
    "middle": [9, 10, 11, 12],
    "ring":   [13, 14, 15, 16],
    "pinky":  [17, 18, 19, 20],
}
ORDER = ["thumb", "index", "middle", "ring", "pinky"]

def _angle(a, b, c):
    """Return angle at point b (radians)."""
    ab = a - b
    cb = c - b
    ab_n = np.linalg.norm(ab) + 1e-6
    cb_n = np.linalg.norm(cb) + 1e-6
    ab = ab / ab_n
    cb = cb / cb_n
    cosang = float(np.clip(np.dot(ab, cb), -1.0, 1.0))
    return float(np.arccos(cosang))

def extract_finger_angles(xyz21):
    """
    xyz21: list/array of 21 (x,y,z) landmarks
    returns 15 angles [thumb 3, index 3, middle 3, ring 3, pinky 3] in radians
    """
    pts = np.asarray(xyz21, dtype=np.float32)
    out = []
    # thumb: wrist(0)-CMC(1)-MCP(2)-IP(3)-TIP(4)
    out.append(_angle(pts[0], pts[1], pts[2]))
    out.append(_angle(pts[1], pts[2], pts[3]))
    out.append(_angle(pts[2], pts[3], pts[4]))
    # other fingers: MCP, PIP, DIP
    WRIST = 0
    for name in ["index", "middle", "ring", "pinky"]:
        f = FINGERS[name]
        out.append(_angle(pts[WRIST], pts[f[0]], pts[f[1]]))  # MCP with proximal anchor
        out.append(_angle(pts[f[0]],   pts[f[1]], pts[f[2]]))  # PIP
        out.append(_angle(pts[f[1]],   pts[f[2]], pts[f[3]]))  # DIP
    return out  # length 15

def angles_to_bends(angles_rad):
    """Convert internal angles to bend magnitudes: bend = pi - angle."""
    bends = []
    for a in angles_rad:
        if a is None:
            bends.append(0.0)
        else:
            bends.append(max(0.0, float(np.pi - a)))
    return bends

def normalize_curls(angles_rad, ranges=None, closed_is_1=True):
    """
    Map 15 joint angles -> five curls in [0..1] for thumb..pinky.
    closed_is_1=True means 1 = closed, 0 = open.
    """
    if not angles_rad or len(angles_rad) < 15:
        return [0.0, 0.0, 0.0, 0.0, 0.0]

    bends = angles_to_bends(angles_rad)

    # tighter defaults; adjust if needed
    default = {
        "thumb":  [(0.00, 0.60), (0.00, 0.70), (0.00, 0.50)],
        "index":  [(0.05, 1.60), (0.05, 1.80), (0.05, 1.30)],
        "middle": [(0.05, 1.60), (0.05, 1.80), (0.05, 1.30)],
        "ring":   [(0.05, 1.60), (0.05, 1.80), (0.05, 1.30)],
        "pinky":  [(0.05, 1.60), (0.05, 1.80), (0.05, 1.30)],
    }
    R = ranges or default

    def clamp01(x):
        return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

    # sensitivity for non-thumb fingers
    SENS_DEAD  = {"index": 0.02, "middle": 0.02, "ring": 0.02, "pinky": 0.02}
    SENS_GAMMA = {"index": 0.70, "middle": 0.70, "ring": 0.70, "pinky": 0.70}
    SENS_GAIN  = {"index": 1.20, "middle": 1.20, "ring": 1.20, "pinky": 1.20}

    curls = []
    k = 0
    for name in ORDER:
        if name not in R or len(R[name]) != 3:
            R[name] = [(0.0, 1.5), (0.0, 1.5), (0.0, 1.5)]

        vals = []
        for j in range(3):
            lo, hi = R[name][j]
            span = max(hi - lo, 1e-6)
            v = (bends[k] - lo) / span
            vals.append(clamp01(v))
            k += 1

        if name == "thumb":
            # simple and responsive
            curl = clamp01(max(vals))
        else:
            # PIP dominates
            curl = 0.20*vals[0] + 0.65*vals[1] + 0.15*vals[2]
            # shaping
            dead  = SENS_DEAD[name]
            gamma = SENS_GAMMA[name]
            gain  = SENS_GAIN[name]
            curl = max(0.0, (curl - dead) / (1.0 - dead))
            curl = curl ** gamma
            curl = clamp01(gain * curl)

        curls.append(clamp01(curl))

    if not closed_is_1:
        curls = [1.0 - c for c in curls]
    return curls

