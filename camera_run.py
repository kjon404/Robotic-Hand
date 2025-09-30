# main_bits.py
import cv2, time

from hand_tracking import HandTracker
from angles import extract_finger_angles, normalize_curls
from viz import draw_hand_skeleton
from serial_sender import SerialHand
from collections import deque

class CurlFilter:
    def __init__(self, alpha=0.40, window=3, max_rate=4.0, deadband=0.015):
        """
        alpha: EMA blend toward a median-filtered target
        window: median window length (small)
        max_rate: max change per second in curl units
        deadband: ignore tiny changes
        """
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
            med = b[len(b)//2]
            if not self.ready:
                y_new = med
            else:
                # EMA toward median
                y_new = self.y[i] + self.alpha * (med - self.y[i])
                # slew limit
                max_step = self.max_rate * max(dt, 1e-3)
                dy = y_new - self.y[i]
                if dy > max_step:   y_new = self.y[i] + max_step
                elif dy < -max_step: y_new = self.y[i] - max_step
                # deadband
                if abs(y_new - self.y[i]) < self.deadband:
                    y_new = self.y[i]
            y_new = self._clamp01(y_new)
            out.append(y_new)
        self.y = out
        self.ready = True
        return out

MODEL = "hand_landmarker.task"
PORT  = "/dev/ttyACM0"   # set to your device, e.g. /dev/ttyUSB0
BAUD  = 115200             # match the baud you just used in your working test


PROCESS_FPS = 15
DRAW_SKELETON = True
SEND_MAX_HZ = 30

def main():
    cv2.setUseOptimized(True)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    tracker = HandTracker(MODEL, max_hands=1)
    hand = SerialHand(port=PORT, baud=BAUD, timeout=0.01, max_hz=SEND_MAX_HZ)
    
    PROCESS_FPS = 20
    last_proc = 0.0
    proc_dt = 1.0 / max(1, PROCESS_FPS)
    last_log = 0.0

        # filters and hold logic
    filt = CurlFilter(alpha=0.40, window=3, max_rate=4.0, deadband=0.015)
    last_seen = time.monotonic()
    last_out = [0.0]*5
    last_send_t = 0.0
    keepalive_dt = 0.25   # force a write every 250 ms
    send_eps = 0.01  
    
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
                
            
                pack = tracker.detect(frame, ts_ms)   # {"hands":[{landmarks, angles_rad, ...}, ...]}
                hands = pack["hands"] if isinstance(pack, dict) and "hands" in pack else []

                if hands:
                 h0 = hands[0]
                 if DRAW_SKELETON:
                     draw_hand_skeleton(frame, h0["landmarks"], color=(0,255,0))

                 lms = h0.get("landmarks")
                 floats= [0.0]*5
                 if lms is not None and len(lms) == 21:
                     try:
                         angles15 = extract_finger_angles(h0["landmarks"])
                         floats = normalize_curls(angles15, ranges=None, closed_is_1=True)
                         last_seen = now
                     except Exception:
                         floats = last_out

                else:
                     floats = last_out if (now - last_seen) < 0.20 else [0.0]*5
                smoothed = filt.step(floats, dt)

                changed = any(abs(smoothed[i] - last_out[i]) > send_eps for i in range(5))
                if changed or (now - last_send_t) > keepalive_dt:
                    hand.send_floats(smoothed)
                    last_out = smoothed
                    last_send_t = now

            cv2.imshow("Vision -> Bits -> Hand", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        tracker.close()
        hand.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

