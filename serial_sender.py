# serial_sender.py
import time, threading, collections
import serial, serial.tools.list_ports
from serial import SerialException, SerialTimeoutException

class SerialHand:
    """
    Nonblocking sender with auto-reconnect.
    - Keeps only the latest packet to avoid backlog
    - Sends at most max_hz to avoid USB backpressure
    - Reconnects on any SerialException or timeout
    """
    def __init__(self, port=None, baud=115200, timeout=0.01, max_hz=40):
        self._cfg = dict(port=port, baud=baud, timeout=timeout, max_hz=max_hz)
        self._ser = None
        self._lock = threading.Lock()
        self._alive = True
        self._q = collections.deque(maxlen=1)
        self._min_dt = 1.0 / max(1, int(max_hz))
        self._last_send = 0.0
        self._port_hint = port
        self._port_opened = None

        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx.start()

    # public API
    def send_floats(self, vals):
        if not isinstance(vals, (list, tuple)) or len(vals) != 5:
            return

        vals = [0.0 if v < 0.0 else 1.0 if v > 1.0 else float(v) for v in vals]   
        line = "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(*vals)
        self._q.append(line.encode("ascii"))

    def close(self):
        self._alive = False
        try:
            self._tx.join(timeout=0.5)
        except Exception:
            pass
        with self._lock:
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None

    # internals
    def _tx_loop(self):
        backoff = 0.5
        while self._alive:
            if not self._ensure_open():
                time.sleep(backoff)
                backoff = min(backoff * 1.5, 3.0)
                continue
            backoff = 0.5

            pkt = self._q[-1] if self._q else None
            now = time.time()
            if pkt is None or (now - self._last_send) < self._min_dt:
                time.sleep(0.001)
                continue

            try:
                with self._lock:
                    if self._ser:
                        self._ser.write(pkt)
                self._last_send = now
            except (SerialTimeoutException, SerialException, OSError):
                self._safe_close()   # will reconnect on next loop
                # do not sleep long, try reconnect soon
                time.sleep(0.2)

    def _safe_close(self):
        with self._lock:
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._port_opened = None

    def _ensure_open(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
        # open outside the lock to avoid long holds
        port = self._select_port()
        if port is None:
            return False
        try:
            ser = serial.Serial(
                port,
                baudrate=self._cfg["baud"],
                timeout=self._cfg["timeout"],
                write_timeout=0.05
            )
            # allow board reset after open
            time.sleep(1.0)
            with self._lock:
                self._ser = ser
                self._port_opened = port
            return True
        except Exception:
            return False

    def _select_port(self):
        # if user provided a fixed path, try that first every time
        if self._port_hint:
            return self._port_hint

        # else find any likely device
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return None
        for p in ports:
            dev = p.device
            if ("USB" in dev) or ("ACM" in dev) or ("COM" in dev):
                return dev
        return ports[0].device

