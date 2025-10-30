# gui_hand_control.py
# Drop-in GUI that uses your existing camera_run pipeline & RFCOMM helpers.

# --- (optional) tame Qt/OpenCV plugin conflicts; safe to keep ---
import os
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
try:
    import PySide6, pathlib
    p = pathlib.Path(PySide6.__file__).with_name("Qt").joinpath("plugins","platforms")
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(p)
except Exception:
    pass

import sys, time
from typing import Optional, List
import numpy as np
import cv2

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QCheckBox,
    QVBoxLayout, QHBoxLayout, QMessageBox
)
from PySide6.QtGui import QImage, QPixmap

# ---- import your existing code (unchanged) ----
# expects camera_run.py alongside this file
import camera_run as CR

def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

# ---------- robust skeleton drawer (accepts normalized or pixel coords) ----------
_MP_EDGES = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (5,9),(9,10),(10,11),(11,12),
    (9,13),(13,14),(14,15),(15,16),
    (13,17),(17,18),(18,19),(19,20),
    (0,17),
]
def draw_skeleton_safe(frame_bgr, landmarks, color=(0,255,0), radius=3, thickness=2):
    """landmarks: 21x2 or 21x3; normalized [0..1] or pixels; draws onto frame_bgr."""
    if landmarks is None:
        return
    pts = np.asarray(landmarks, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] != 21 or pts.shape[1] < 2:
        return
    h, w = frame_bgr.shape[:2]
    # If values look normalized, scale to pixels
    if float(pts[:, :2].max()) <= 1.2:
        pts[:, 0] *= w
        pts[:, 1] *= h
    pts = pts[:, :2].astype(int)
    for i, j in _MP_EDGES:
        cv2.line(frame_bgr, tuple(pts[i]), tuple(pts[j]), color, thickness, cv2.LINE_AA)
    for p in pts:
        cv2.circle(frame_bgr, tuple(p), radius, color, -1, cv2.LINE_AA)

# ---------------- Worker Thread ----------------
class VisionWorker(QThread):
    frame_ready = Signal(object)        # BGR numpy array (already drawn on)
    floats_ready = Signal(list)         # five floats 0..1
    status = Signal(str)

    def __init__(self, cam_index: int, width: int, height: int, fps_req: int,
                 draw_skeleton: bool):
        super().__init__()
        self.cam_index = cam_index
        self.width = width
        self.height = height
        self.fps_req = fps_req
        self.draw_skel = draw_skeleton

        self._stop = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.bt: Optional[CR.BluetoothBin] = None
        self.tracker: Optional[CR.HandTracker] = None
        self.curls_engine: Optional[CR.HandCurls] = None

        # Filters / state from your code
        self.filt = CR.CurlFilter(alpha=0.25, window=5, max_rate=8.0, deadband=0.006)
        self.last_seen = 0.0
        self.last_out = ([1.0]*5 if CR.OPEN_IS_1 else [0.0]*5)[:]  # start "open" for priming
        self.keepalive_dt = 0.25
        self.send_eps = 0.003
        self.last_send_t = 0.0
        self.process_fps = max(1, int(getattr(CR, "PROCESS_FPS", 20)))
        self.send_max_hz = int(getattr(CR, "SEND_MAX_HZ", 30))

    def stop(self): self._stop = True

    def _open_camera(self) -> bool:
        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_ANY)
        if self.width and self.height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps_req:
            cap.set(cv2.CAP_PROP_FPS, self.fps_req)
        try:
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        ok, _ = cap.read()
        if not ok:
            cap.release()
            return False
        self.cap = cap
        return True

    def _open_bt(self):
        CR.ensure_rfcomm_connected(
            CR.RECEIVER_MAC,
            idx=CR.RFCOMM_INDEX,
            channels=CR.TRY_CHANNELS,
            use_sudo=CR.USE_SUDO,
            timeout=25
        )
        self.bt = CR.BluetoothBin(port=CR.RFCOMM_DEV, max_hz=self.send_max_hz)
        # Prime a few "open" frames
        open_vec = [1.0]*5 if CR.OPEN_IS_1 else [0.0]*5
        for _ in range(10):
            self.bt.send_floats(open_vec)
            time.sleep(0.02)

    def run(self):
        try:
            self.status.emit("Opening camera…")
            if not self._open_camera():
                self.status.emit("Camera failed to open")
                return

            self.status.emit("Loading hand tracker…")
            self.tracker = CR.HandTracker(CR.MODEL, max_hands=1)
            self.curls_engine = CR.HandCurls()

            self.status.emit("Connecting RFCOMM…")
            self._open_bt()
            self.status.emit("Streaming…")

            proc_dt = 1.0 / float(self.process_fps)
            last_proc = 0.0

            while not self._stop:
                ok, frame = self.cap.read()
                if not ok:
                    self.status.emit("Camera read failed")
                    break

                frame = cv2.flip(frame, 1)

                now = time.monotonic()
                if last_proc == 0.0 or (now - last_proc) >= proc_dt:
                    dt = proc_dt if last_proc == 0 else (now - last_proc)
                    last_proc = now
                    ts_ms = int(now * 1000)

                    floats = self.last_out
                    try:
                        pack = self.tracker.detect(frame, ts_ms)
                        hands = pack.get("hands", []) if isinstance(pack, dict) else []
                        if hands:
                            h0 = hands[0]

                            # --- overlay: always draw using 2D image landmarks ---
                            if self.draw_skel:
                                lm2d = h0.get("landmarks")
                                if lm2d is not None and len(lm2d) == 21:
                                    draw_skeleton_safe(frame, lm2d, color=(0,255,0))

                            # --- curls: prefer 3D/world if present, else 2D landmarks ---
                            lms = h0.get("world", h0.get("landmarks"))
                            if lms is not None and len(lms) == 21:
                                L = np.asarray(lms, dtype=np.float32).reshape(21, 3)
                                curls = (CR.simple_curls_from_L(L) if CR.USE_SIMPLE_CURLS
                                         else self.curls_engine.curls5(L))
                                floats = ((1.0 - curls) if CR.OPEN_IS_1 else curls).tolist()
                                self.last_seen = now
                            else:
                                floats = self.last_out
                        else:
                            # hold last for 0.35s, then go to "open"
                            floats = self.last_out if (now - self.last_seen) < 0.35 else (
                                [1.0]*5 if CR.OPEN_IS_1 else [0.0]*5
                            )
                    except Exception as e:
                        self.status.emit(f"Tracker error: {e}")
                        floats = self.last_out

                    smoothed = floats if CR.BYPASS_FILTER else self.filt.step(floats, dt)
                    smoothed = [_clamp01(v) for v in smoothed]

                    # emit to GUI
                    self.floats_ready.emit(list(smoothed))
                    # update preview (drawn frame)
                    self.frame_ready.emit(frame)

                    changed = any(abs(smoothed[i] - self.last_out[i]) > self.send_eps for i in range(5))
                    if self.bt and (changed or (now - self.last_send_t) > self.keepalive_dt):
                        try:
                            self.bt.send_floats(smoothed)
                            self.last_send_t = now
                            self.last_out = smoothed
                        except Exception as e:
                            self.status.emit(f"BT send error: {e}")

            self.status.emit("Stopping…")

        finally:
            try:
                if self.tracker: self.tracker.close()
            except: pass
            try:
                if self.bt: self.bt.close()
            except: pass
            try:
                if self.cap:
                    self.cap.release(); self.cap = None
            except: pass
            try:
                CR.graceful_disconnect()
            except: pass

# ---------------- GUI ----------------
class MainUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera → RFCOMM (drop-in GUI)")
        self.setMinimumSize(1000, 720)

        # Preview
        self.preview = QLabel("Preview")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(900, 540)

        # Camera selectors
        self.cam_combo = QComboBox()
        self.scan_btn  = QPushButton("Scan Cameras")
        self.res_combo = QComboBox(); self.res_combo.addItems(["640x480", "1280x720", "1920x1080"])
        self.fps_combo = QComboBox(); self.fps_combo.addItems(["30", "60"])
        self.draw_chk  = QCheckBox("Draw skeleton")
        self.draw_chk.setChecked(bool(getattr(CR, "DRAW_SKELETON", True)))

        # BT info (from your config)
        self.bt_label = QLabel(
            f"Pi MAC: <b>{CR.RECEIVER_MAC}</b> | RFCOMM idx: <b>{CR.RFCOMM_INDEX}</b> | Dev: <b>{CR.RFCOMM_DEV}</b>"
        )

        self.start_btn = QPushButton("Start")
        self.stop_btn  = QPushButton("Stop")

        self.status = QLabel("Status: idle")
        self.values = QLabel("Floats: [_, _, _, _, _]")

        # Layout
        root = QVBoxLayout(self)
        root.addWidget(self.preview, 1)

        r1 = QHBoxLayout()
        r1.addWidget(self.cam_combo, 1)
        r1.addWidget(self.scan_btn)
        r1.addWidget(QLabel("Res")); r1.addWidget(self.res_combo)
        r1.addWidget(QLabel("FPS")); r1.addWidget(self.fps_combo)
        r1.addWidget(self.draw_chk)
        root.addLayout(r1)

        r2 = QHBoxLayout()
        r2.addWidget(self.start_btn)
        r2.addWidget(self.stop_btn)
        r2.addStretch(1)
        root.addLayout(r2)

        root.addWidget(self.bt_label)
        root.addWidget(self.values)
        root.addWidget(self.status)

        # Wire
        self.scan_btn.clicked.connect(self.scan_cameras)
        self.start_btn.clicked.connect(self.start)
        self.stop_btn.clicked.connect(self.stop)
        self.draw_chk.stateChanged.connect(self.on_draw_toggle)

        self.worker: Optional[VisionWorker] = None
        self.scan_cameras()

        # simple UI refresher to keep label layout snappy
        self._ui_timer = QTimer(self); self._ui_timer.timeout.connect(lambda: None); self._ui_timer.start(250)

    # ---- camera scan ----
    def scan_cameras(self):
        self.cam_combo.clear()
        found = False
        for i in range(10):
            cap = cv2.VideoCapture(i, cv2.CAP_ANY)
            ok, _ = cap.read()
            if ok:
                self.cam_combo.addItem(f"Camera {i}", i)
                found = True
            cap.release()
        self.status.setText("Cameras scanned" if found else "No cameras found")

    # ---- live toggle for overlay ----
    def on_draw_toggle(self, _):
        if self.worker:
            self.worker.draw_skel = self.draw_chk.isChecked()

    # ---- start/stop ----
    def start(self):
        if self.worker and self.worker.isRunning():
            QMessageBox.information(self, "Already running", "Stop first, then start again.")
            return
        if self.cam_combo.count() == 0:
            QMessageBox.warning(self, "No camera", "No camera detected.")
            return

        idx = self.cam_combo.currentData()
        try:
            w, h = map(int, self.res_combo.currentText().split("x"))
        except:
            w, h = 640, 480
        try:
            fps = int(self.fps_combo.currentText())
        except:
            fps = 30

        self.worker = VisionWorker(
            cam_index=int(idx), width=w, height=h, fps_req=fps,
            draw_skeleton=self.draw_chk.isChecked()
        )
        self.worker.frame_ready.connect(self.on_frame)
        self.worker.floats_ready.connect(self.on_floats)
        self.worker.status.connect(self.on_status)
        self.worker.start()
        self.status.setText("Starting…")

    def stop(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None
            self.status.setText("Stopped")

    # ---- event handlers ----
    @Slot(object)
    def on_frame(self, frame_bgr):
        if frame_bgr is None: return
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
        self.preview.setPixmap(QPixmap.fromImage(img))

    @Slot(list)
    def on_floats(self, vals: List[float]):
        self.values.setText("Floats: [" + ", ".join(f"{v:.3f}" for v in vals) + "]")

    @Slot(str)
    def on_status(self, msg: str):
        self.status.setText(msg)

    # ---- cleanup ----
    def closeEvent(self, e):
        try:
            self.stop()
            CR.graceful_disconnect()
        except Exception:
            pass
        e.accept()

# ---------------- main ----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainUI()
    w.show()
    sys.exit(app.exec())

