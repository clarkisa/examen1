#!/usr/bin/env python3
# rpi_wasd_v4.py — Control USB-CDC para v4 (WASD hold, b, 0/SPACE, k/l duty)

import sys, termios, tty, select, time, glob
import serial

BAUD = 115200
READ_TIMEOUT = 0.1

def autodetect_port():
    # macOS
    mac = sorted(glob.glob("/dev/tty.usbmodem*")) or sorted(glob.glob("/dev/tty.usbserial*"))
    if mac: return mac[0]
    # Linux/Raspberry
    lin = sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*"))
    if lin: return lin[0]
    raise RuntimeError("No serial device found (/dev/tty.usbmodem*, /dev/ttyACM*, /dev/ttyUSB*)")

class RawKB:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)  # sin canon, sin eco
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def getch(self, timeout=0.02):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None

def send(ser, ch):
    ser.write(ch.encode("utf-8"))
    ser.flush()

HELP = """\
Controls (hold to move):
  w/s/a/d  → forward / reverse / left / right
  SPACE/0  → stop
  b        → buzzer (2s)
  k / l    → duty down / up  (levels: 5,25,50,75,100)
Exit:
  q / ESC / Ctrl+C
"""

def main():
    try:
        port = autodetect_port()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    print(f"[INFO] Opening {port} @ {BAUD}")
    try:
        ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)
    except Exception as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        sys.exit(1)

    print("[INFO] Hold WASD to move; SPACE/0=stop; b=buzzer; k/l duty; q/ESC=quit.")
    print(HELP)

    last_key = None
    last_time = 0.0
    REPEAT_HZ = 8.0
    REPEAT_DT = 1.0 / REPEAT_HZ
    HOLD_TIMEOUT = 0.25
    last_repeat = 0.0

    def press(k):
        nonlocal last_key, last_time, last_repeat
        k = k.lower()
        if k in ('\x1b', 'q', '\x03'):  # Esc / q / Ctrl+C
            print("\nBye!"); raise KeyboardInterrupt
        if k in (' ', '0', 'b', 'k', 'l', 'w', 'a', 's', 'd'):
            send(ser, k)
            if k in ('w','a','s','d'):
                last_key, last_time, last_repeat = k, time.monotonic(), time.monotonic()

    try:
        with RawKB() as kb, ser:
            while True:
                now = time.monotonic()
                ch = kb.getch(timeout=0.02)
                if ch:
                    press(ch)

                # autorepeat de WASD mientras mantienes
                if last_key is not None:
                    if (now - last_time) > HOLD_TIMEOUT:
                        last_key = None
                    elif (now - last_repeat) >= REPEAT_DT:
                        send(ser, last_key)
                        last_repeat = now

                # lee y muestra respuestas de la Tiva (STATE/DUTY/DIST/…)
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print("\r" + line + " " * 12, end="", flush=True)
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()