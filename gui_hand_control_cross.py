# gui_hand_control_cross.py
# Cross-platform GUI that uses your camera_run pipeline.
# - Linux/macOS: uses camera_run.ensure_rfcomm_connected + BluetoothBin (RFCOMM device)
# - Windows: uses COM port + pyserial (no rfcomm), same 26-byte frame

# --- optional: tame Qt/OpenCV plugin path issues ---
import os
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import sys, time, struct
from typing import Optional, List
import numpy as np
import cv2

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QCheckBox,
    QVBoxLayout, QHBoxLayout, QMessageBox
)
from PySide6.QtGui import QImage, QPixmap

import platform
IS_WINDOWS = (os.name == "nt" or platform.system().lower() == "windows")

# ---- import your existing code (unchanged) ----
import camera_run as CR

# ---------- Windows COM sender (identical frame format) ----------
if IS_WINDOWS:
    import serial, serial.tools.list_ports

    SYNC = b"\xA5\x5A"
    FRAME_LEN = 26
    def _checksum16(b: bytes) -> int: return sum(b) & 0xFFFF

    class BluetoothBinWin:
        """Windows sender over COMx (SPP). Same API as CR.BluetoothBin: send_floats(vals5)."""
        def __init__(self, port: str, max_hz: int = 30):
            self.min_interval = 1.0 / float(max_hz) if max_hz else 0.0
            self.last_send = 0.0
            self.seq = 0
            # baud is ignored by BT SPP, but pyserial needs a value
            self.ser = serial.Serial(port=port, baudrate=115200, timeout=0)

        def send_floats(self, vals5):
            now = time.time()
            if self.min_interval and (now - self.last_send) < self.min_interval:
                return
            payload = struct.pack("<5f", *[float(v) for v in vals5])
            pkt = SYNC + struct.pack("<H", self.seq & 0xFFFF) + payload + struct.pack("<H", _checksum16(payload))
            self.ser.write(pkt); self.ser.flush()
            self.last_send = now
            self.seq = (self.seq + 1) & 0xFFFF

        def close(self):
            try: self.ser.close()
            except: pass

    def pick_windows_bt_com(pi_mac: Optional[str] = None) -> Optional[str]:
        """Try to auto-pick the Outgoing SPP COM port for your Pi."""
        mac_plain = pi_mac.replace(":", "").upper() if pi_mac else None
        cand = []
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").upper()
            # Heuristics: Bluetooth serial, prefer 'Outgoing'
            if ("bluetooth" in desc) or ("serial over bluetooth" in desc) or ("standard serial over bluetooth" in desc):
                score = 1
                if "outgoing" in desc: score += 3
                if mac_plain and mac_plain in hwid: score += 5
                cand.append((score, p.device))
        if not cand:
            return None
        cand.sort(reverse=True)
        return cand[0][1]
else:
    serial = None  # not needed


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

# ---------- robust skeleton drawer ----------
_MP_EDGES = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (5,9),(9,10),(10,11),(11,12),
    (9,13),(13,14),(14,15),(15,16),
    (13,17),(17,18),(18,19),(19,20),
    (0,17),
]
def draw_skeleton_safe(frame_bgr, landmarks, color=(0,255,0), radius=3, thickness=2):
    """landmarks: 21x2 or 21x3; normalized [0..1] or pixels."""
    if landmarks is None: return
    pts = np.asarray(landmarks, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] != 21 or pts.shape[1] < 2: return
    h, w = frame_bgr.shape[:2]
    if float(pts[:, :2].max()) <= 1.2:  # normalized
        pts[:, 0] *= w; pts[:, 1] *= h
    pts = pts[:, :2].astype(int)
    for i, j in _MP_EDGES:
        cv2.line(frame_bgr, tuple(pts[i]), tuple(pts[j]), color, thickness, cv2.LINE_AA)
    for p in pts:
        cv2.circle(frame_bgr, tuple(p), radius, color, -1, cv2.LINE_AA)

# ---------------- Worker Thread ----------------
class VisionWorker(QThread):
    frame_ready = Signal(object)        # BGR numpy array
    floats_ready = Signal(list)         # five floats 0..1
    status = Signal(str)

    def __init__(self, cam_index: int, width: int, height: int, fps_req: int,
                 draw_skeleton: bool,
                 win_port: Optional[str]):  # only used on Windows
        super().__init__()
        self.cam_index = cam_index
        self.width = width
        self.height = height
        self.fps_req = fps_req
        self.draw_skel = draw_skeleton
        self.win_port = win_port

        self._stop = False
        self.cap: Optional[cv2.VideoCapture] = None
        self.bt = None  # CR.BluetoothBin or BluetoothBinWin
        self.tracker: Optional[CR.HandTracker] = None
        self.curls_engine: Optional[CR.HandCurls] = None

        # Filters / state (from your code)
        self.filt = CR.CurlFilter(alpha=0.25, window=5, max_rate=8.0, deadband=0.006)
        self.last_seen = 0.0
        self.last_out = ([1.0]*5 if CR.OPEN_IS_1 else [0.0]*5)[:]
        self.keepalive_dt = 0.25
        self.send_eps = 0.003
        self.last_send_t = 0.0
        self.process_fps = int(getattr(CR, "PROCESS_FPS", 20))
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
        if IS_WINDOWS:
            # Windows: user-picked COM port, or auto-pick from MAC
            port = self.win_port or (pick_windows_bt_com(CR.RECEIVER_MAC) if 'RECEIVER_MAC' in dir(CR) else None)
            if not port:
                raise RuntimeError("No Bluetooth COM port found. Pair the Pi and select the Outgoing COM port.")
            self.bt = BluetoothBinWin(port=port, max_hz=self.send_max_hz)
        else:
            # Linux/macOS: use your existing rfcomm path
            CR.ensure_rfcomm_connected(
                CR.RECEIVER_MAC,
                idx=CR.RFCOMM_INDEX,
                channels=CR.TRY_CHANNELS,
                use_sudo=CR.USE_SUDO,
                timeout=25
            )
            self.bt = CR.BluetoothBin(port=CR.RFCOMM_DEV, max_hz=self.send_max_hz)

        # Prime a few "open" frames so Teensy is in a known state
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

            self.status.emit("Opening Bluetooth link…")
            self._open_bt()
            self.status.emit("Streaming…")

            proc_dt = 1.0 / float(max(1, self.process_fps))
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
                            if self.draw_skel:
                                lm2d = h0.get("landmarks")
                                if lm2d is not None and len(lm2d) == 21:
                                    draw_skeleton_safe(frame, lm2d, color=(0,255,0))
                            lms = h0.get("world", h0.get("landmarks"))
                            if lms is not None and len(lms) == 21:
                                L = np.asarray(lms, dtype=np.float32).reshape(21, 3)
                                curls = (CR.simple_curls_from_L(L) if getattr(CR, "USE_SIMPLE_CURLS", False)
                                         else self.curls_engine.curls5(L))
                                floats = ((1.0 - curls) if CR.OPEN_IS_1 else curls).tolist()
                                self.last_seen = now
                            else:
                                floats = self.last_out
                        else:
                            floats = self.last_out if (now - self.last_seen) < 0.35 else (
                                [1.0]*5 if CR.OPEN_IS_1 else [0.0]*5
                            )
                    except Exception as e:
                        self.status.emit(f"Tracker error: {e}")
                        floats = self.last_out

                    smoothed = floats if getattr(CR, "BYPASS_FILTER", False) else self.filt.step(floats, dt)
                    smoothed = [_clamp01(v) for v in smoothed]

                    # emit to GUI
                    self.floats_ready.emit(list(smoothed))
                    self.frame_ready.emit(frame)

                    changed = any(abs(smoothed[i] - self.last_out[i]) > 0.003 for i in range(5))
                    if self.bt and (changed or (now - self.last_send_t) > 0.25):
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
        self.setWindowTitle("Camera → RFCOMM (Windows & Linux)")
        self.setMinimumSize(1000, 720)

        # Preview
        self.preview = QLabel("Preview"); self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(900, 540)

        # Camera selectors
        self.cam_combo = QComboBox()
        self.scan_btn  = QPushButton("Scan Cameras")
        self.res_combo = QComboBox(); self.res_combo.addItems(["640x480", "1280x720", "1920x1080"])
        self.fps_combo = QComboBox(); self.fps_combo.addItems(["30", "60"])
        self.draw_chk  = QCheckBox("Draw skeleton"); self.draw_chk.setChecked(bool(getattr(CR, "DRAW_SKELETON", True)))

        # Bluetooth info
        bt_text = f"Pi MAC: <b>{getattr(CR, 'RECEIVER_MAC', '??')}</b>"
        if IS_WINDOWS:
            bt_text += " | Pick COM (Windows SPP)"
        else:
            bt_text += f" | RFCOMM idx: <b>{getattr(CR, 'RFCOMM_INDEX','0')}</b> | Dev: <b>{getattr(CR,'RFCOMM_DEV','/dev/rfcomm0')}</b>"
        self.bt_label = QLabel(bt_text)

        # Windows-only: COM picker
        self.win_com_combo = QComboBox()
        self.win_scan_com_btn = QPushButton("Scan COMs")
        self.win_row = QHBoxLayout()
        if IS_WINDOWS:
            self.win_row.addWidget(self.win_com_combo, 1)
            self.win_row.addWidget(self.win_scan_com_btn)

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

        if IS_WINDOWS:
            root.addLayout(self.win_row)

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
        if IS_WINDOWS:
            self.win_scan_com_btn.clicked.connect(self.scan_coms)

        self.worker: Optional[VisionWorker] = None
        self.scan_cameras()
        if IS_WINDOWS: self.scan_coms()

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

    # ---- Windows COM scan ----
    def scan_coms(self):
        if not IS_WINDOWS: return
        import serial.tools.list_ports
        self.win_com_combo.clear()
        # Preselect a likely COM for the Pi
        auto = None
        try:
            auto = pick_windows_bt_com(getattr(CR, "RECEIVER_MAC", None))
        except Exception:
            auto = None
        for p in serial.tools.list_ports.comports():
            label = f"{p.device} — {p.description}"
            self.win_com_combo.addItem(label, p.device)
        if auto:
            idx = self.win_com_combo.findData(auto)
            if idx >= 0:
                self.win_com_combo.setCurrentIndex(idx)
        self.status.setText("COM ports scanned (Windows)")

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

        win_port = None
        if IS_WINDOWS:
            win_port = self.win_com_combo.currentData()
            if not win_port:
                QMessageBox.warning(self, "No COM", "Pick the Outgoing COM port for the Pi (pair in Bluetooth settings).")
                return

        self.worker = VisionWorker(
            cam_index=int(idx), width=w, height=h, fps_req=fps,
            draw_skeleton=self.draw_chk.isChecked(),
            win_port=win_port
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

