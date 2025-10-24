# camera_run.py  (drop-in)
import cv2, time
import numpy as np
import os, time, socket, struct, threading, queue

from hand_tracking import HandTracker
from angles import HandCurls
from viz import draw_hand_skeleton
from serial_sender import SerialHand
from collections import deque

# === knobs you may change ===
MODEL = "hand_landmarker.task"
PORT  = "/dev/ttyACM0"
BAUD  = 115200

#task to connect via rfcomm
os.system("sudo rfcomm connect hci0 B8:27:EB:42:DE:29 3 &")
time.sleep(2)

print("Connected via rfcomm")


PROCESS_FPS   = 20
SEND_MAX_HZ   = 30
DRAW_SKELETON = True

OPEN_IS_1     = False         # False means 0 open, 1 closed
DEBUG_OVERLAY = True 
BYPASS_FILTER = False          # set True to test without smoothing
USE_SIMPLE_CURLS = False        # start True for full-range mapping without calibration


#BLE SENDER

PI_BT_MAC = ""
RFCOMM_PORT = 3

class BluetoothSender(threading.Thread):
    def __init__(self,addr, port=RFCOMM_PORT, max_hz=30):
        super().__init__(daemon=True)
        self.addr = addr
        self.port = port
        self.period = 1.0 / float(max_hz) if max_hz else 0.0



# ---------- curl math (simple, no calibration) ----------
# Use MCP for thumb and PIP for the other fingers
_TRIPLETS = np.array([
    [0, 1, 2],    # thumb MCP
    [5, 6, 7],    # index PIP
    [9, 10, 11],  # middle PIP
    [13, 14, 15], # ring PIP
    [17, 18, 19], # pinky PIP
], dtype=np.int32)

# Open and closed angle targets in degrees, generous for typical geometry
_MIN_DEG = np.array([0.02, 0.04, 0.04, 0.04, 0.04], dtype=np.float32)    # open
_MAX_DEG = np.array([0.70,0.80,0.80,0.80,0.80], dtype=np.float32)   # closed

def _unit(v, eps=1e-9):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, eps)

def simple_curls_from_L(L21):
    L = np.asarray(L21, dtype=np.float32)
    a = L[_TRIPLETS[:, 0]]; b = L[_TRIPLETS[:, 1]]; c = L[_TRIPLETS[:, 2]]
    u1 = _unit(a - b); u2 = _unit(c - b)
    dots = np.clip(np.sum(u1 * u2, axis=1), -1.0, 1.0)
    

    ang = np.arccos(dots)
    bend = np.pi - ang
    curls = (bend - _MIN_DEG) / np.maximum((_MAX_DEG - _MIN_DEG), 1e-5)
    return np.clip(curls, 0.0, 1.0).astype(np.float32)

#Bluetooth sender 

# Drop-in replacement for BluetoothCsv on the laptop

class BluetoothBin:

    SYNC = b"\xA5\x5A"

    def __init__(self, port="/dev/rfcomm0", max_hz=30):
        self.port = port
        self.min_interval = 1.0 / float(max_hz) if max_hz else 0.0
        self.last_send = 0.0
        self.seq = 0

        print("Waiting for", port)
        while not os.path.exists(port):
            time.sleep(0.2)

        # Open non-blocking so camera loop never freezes
        self.fd = os.open(port, os.O_RDWR | os.O_NONBLOCK)
        print("BT port open (non-blocking):", port)

    @staticmethod
    def _csum16(b : bytes) -> int:
        return sum(b) & 0xFFFF

    def send_floats(self, vals5):
        # Only rate-limit by time, never sleep here
        now = time.time()
        if self.min_interval and (now - self.last_send) < self.min_interval:
            return
        payload = struct.pack("<5f", *[float(v) for v in vals5])
        pkt = (
                self.SYNC +
                struct.pack("<h", self.seq & 0xFFFF) +
                payload +
                struct.pack("<H", self._csum16(payload))
            )

        try:
            os.write(self.fd, pkt)   # non-blocking write
            self.last_send = now
            self.seq = (self.seq + 1) & 0xFFFF
        except BlockingIOError:
            # TX buffer full this frame, skip it
            pass

    def close(self):
        try:
            os.close(self.fd)
        except:
            pass


# ---------- adaptive scaler so you always get 0..1 ----------
class AdaptiveRange:
    """
    Per finger adaptive min and max with a small margin.
    Expands whatever subrange the curls live in to the full [0..1] without jumps.
    """
    def __init__(self, low_seed=0.0, high_seed=1.0, alpha=0.02, margin=0.0):
        self.low  = np.full(5, float(low_seed),  dtype=np.float32)
        self.high = np.full(5, float(high_seed), dtype=np.float32)
        self.alpha = float(alpha)
        self.margin = float(margin)

    def step(self, x):
        x = np.asarray(x, dtype=np.float32)
        t_low  = np.minimum(self.low,  x)
        t_high = np.maximum(self.high, x)
        self.low  += self.alpha * (t_low  - self.low)
        self.high += self.alpha * (t_high - self.high)
        self.high = np.maximum(self.high, self.low + 1e-5)
        span = self.high - self.low
        lo = self.low  + self.margin * span
        hi = self.high - self.margin * span
        span = np.maximum(hi - lo, 1e-5)
        y = (x - lo) / span
        return np.clip(y, 0.0, 1.0)

# ---------- optional smoother ----------
class CurlFilter:
    def __init__(self, alpha=0.25, window=5, max_rate=8.0, deadband=0.006):
        self.alpha = alpha
        self.max_rate = max_rate
        self.deadband = deadband
        self.buf = [deque(maxlen=window) for _ in range(5)]
        self.y = [0.0] * 5
        self.ready = False

    @staticmethod
    def _clamp01(x):
        return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

    def step(self, x, dt):
        out = []
        for i in range(5):
            self.buf[i].append(float(x[i]))
            b = list(self.buf[i]); b.sort()
            med = b[len(b) // 2]
            if not self.ready:
                y_new = med
            else:
                y_new = self.y[i] + self.alpha * (med - self.y[i])
                max_step = self.max_rate * max(dt, 1e-3)
                dy = y_new - self.y[i]
                if dy > max_step:   y_new = self.y[i] + max_step
                elif dy < -max_step: y_new = self.y[i] - max_step
                if abs(y_new - self.y[i]) < self.deadband:
                    y_new = self.y[i]
            y_new = self._clamp01(y_new)
            out.append(y_new)
        self.y = out
        self.ready = True
        return out

def main():
    cv2.setUseOptimized(True)
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    tracker = HandTracker(MODEL, max_hands=1)
    #hand = SerialHand(port=PORT, baud=BAUD, timeout=0.01, max_hz=SEND_MAX_HZ)
    #bluetooth
    hand = BluetoothBin(port="/dev/rfcomm0", max_hz=SEND_MAX_HZ)
    curls_engine = HandCurls()

    #Defaults camera if not detected

    if not cap.isOpened():
        print("Camera not found defaulting to 0")
        cap.release()
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # choose open/closed vectors once
    if OPEN_IS_1:
        OPEN_VEC, CLOSED_VEC = [1.0]*5, [0.0]*5
    else:
        OPEN_VEC, CLOSED_VEC = [0.0]*5, [1.0]*5

    
    # timing
    last_proc = 0.0
    proc_dt = 1.0 / max(1, PROCESS_FPS)

    # signal path
    filt = CurlFilter(alpha=0.25, window=5, max_rate=8.0, deadband=0.006)
    adapt = AdaptiveRange(low_seed=0.0, high_seed=1.0, alpha=0.02, margin=0.0)

    # send control
    last_seen = time.monotonic()
    last_out = OPEN_VEC[:]         # start open
    last_send_t = 0.0
    keepalive_dt = 0.25
    send_eps = 0.003

    # send an initial open
    for _ in range(15):
        hand.send_floats(OPEN_VEC)
        time.sleep(0.02)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break

            frame = cv2.flip(frame, 1)
            now = time.monotonic()
            ts_ms = int(now * 1000)
            do_process = (now - last_proc) >= proc_dt

            if do_process:
                dt = now - last_proc if last_proc > 0 else proc_dt
                last_proc = now

                pack = tracker.detect(frame, ts_ms)
                hands = pack["hands"] if isinstance(pack, dict) and "hands" in pack else []

                if hands:
                    h0 = hands[0]
                    if DRAW_SKELETON:
                        lm_draw = h0["landmarks"].tolist() if isinstance(h0["landmarks"], np.ndarray) else h0["landmarks"]
                        draw_hand_skeleton(frame, lm_draw, color=(0, 255, 0))

                    lms = h0.get("world", h0.get("landmarks"))
                    if lms is not None and len(lms) == 21:
                        try:
                            L = np.asarray(lms, dtype=np.float32).reshape(21, 3)

                            # raw curls in 0..1 (0 open, 1 closed)
                            if USE_SIMPLE_CURLS:
                                curls = simple_curls_from_L(L)
                            else:
                                curls = curls_engine.curls5(L)

                            # optional adaptive scaling (kept off per your last setting)
#                            curls = adapt.step(curls)

                            floats = (1.0 - curls).tolist() if OPEN_IS_1 else curls.tolist()
                            last_seen = now
                        except Exception as e:
                            print(f"[angles error] {e}")
                            floats = last_out
                    else:
                        floats = last_out
                else:
                    # no hand: hold briefly, then go to OPEN
                    floats = last_out if (now - last_seen) < 0.35 else OPEN_VEC

                # smoothing
                smoothed = floats if BYPASS_FILTER else filt.step(floats, dt)

                # debug overlay
                if DEBUG_OVERLAY:
                    txt = " ".join(f"{v:0.2f}" for v in smoothed)
                    cv2.putText(frame, txt, (10, 30), cv2.FONT_ITALIC, 0.8, (255, 0, 0), 2)

                # send when changed or keepalive
                changed = any(abs(smoothed[i] - last_out[i]) > send_eps for i in range(5))
                if changed or (now - last_send_t) > keepalive_dt:
                    try:
                        hand.send_floats(smoothed)
                    except Exception as e:
                        print(f"[serial error] {e}")
                    last_out = smoothed
                    last_send_t = now

            cv2.imshow("Vision -> Floats -> Hand", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        tracker.close()
        hand.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ =="__main__":
    main()
