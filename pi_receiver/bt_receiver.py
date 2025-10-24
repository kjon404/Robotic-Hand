#!/usr/bin/env python3
import os, time, sys, struct, select, errno

RFDEV = "/dev/rfcomm0"
SYNC  = b"\xA5\x5A"
FRAME_LEN = 26  # 2 sync + 2 seq + 20 payload + 2 csum


SERIAL_OUT = "/dev/serial0"
BAUD = 115200


def wait_for_rfcomm(dev):
    while not os.path.exists(dev):
        time.sleep(0.2)

def checksum16(b: bytes) -> int:
    return sum(b) & 0xFFFF

def find_sync(fd):
    """Hunt for 0xA5 0x5A."""
    win = b""
    while True:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r: 
            continue
        b = os.read(fd, 1)
        if not b:
            return False
        win = (win + b)[-2:]
        if win == SYNC:
            return True

def read_exact(fd, n):
    buf = bytearray()
    while len(buf) < n:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r: 
            continue
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

def read_exact(fd, n):
    buf = bytearray()
    while len(buf) < n:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r: continue
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def open_serial():
    os.system(f"stty -F {SERIAL_OUT} {BAUD} cs8 -cstopb -parenb -ixon -ixoff -crtscts")
    fd = os.open(SERIAL_OUT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    print(f"[+] Serial opened: {SERIAL_OUT} @ {BAUD}")
    return fd

def clamp01(v): 
    return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v


def main():
    print("[*] Waiting for Bluetooth serial…")
    wait_for_rfcomm(RFDEV)
    ser_fd = open_serial()

    while True:
        try:
            fd = os.open(RFDEV, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            print("[*] Connected. Relaying to serial")
            while True:
                if not find_sync(fd):
                    break
                rest = read_exact(fd, FRAME_LEN - 2)
                if not rest or len(rest) != (FRAME_LEN - 2):
                    break
                # rest layout: seq(2) + payload(20) + csum(2)
                seq, = struct.unpack_from("<H", rest, 0)
                payload = rest[2:22]
                csum_rx, = struct.unpack_from("<H", rest, 22)
                if checksum16(payload) != csum_rx:
                    # bad frame; resync
                    continue
                
                f0, f1, f2, f3, f4 = struct.unpack("<5f", payload)
                vals = [clamp01(v) for v in (f0, f1, f2, f3, f4)]

                line = ("{:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n".format(*vals)).encode("utf-8")
                
                try:
                    os.write(ser_fd, line)
                except OSError as e:
                    if e.errno in (errno.EIO, errno.EPIPE, errno.ENODEV, errno.EBADF):
                        try:
                            os.close(ser_fd)
                        except Exception:
                            pass
                        ser_fd = open_serial()
                    else:
                        raise
        except OSError as e:
            print(f"[-] BT link lost ({e}). Re-waiting")
            wait_for(RFDEV)
        finally:
            try: os.close(bt_fd)
            except Exception: pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        sys.exit(0)

