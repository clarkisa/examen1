#!/usr/bin/env python3
# rpi_send_tiva_v5.py — Control v5 por teclado (sin Enter) + HUD de Velocidad y Distancia
# Teclas:
#   w/s/a/d → mover (persisten hasta que envíes STOP)
#   SPACE o 0 → STOP
#   k / l   → duty - / +
#   b       → buzzer (2 s en la Tiva)
#   h       → ayuda
#   q / Esc / Ctrl+C → salir

import sys, termios, tty, select, time, glob, threading
import serial

BAUD = 115200

HELP = """\
Controles:
  w  → forward
  s  → reverse
  a  → left
  d  → right
  [espacio] o 0 → STOP
  k / l → duty - / +
  b → buzzer (2 s)
  h → ayuda
  q / Esc / Ctrl+C → salir
"""

def autodetect_port():
    # Prioriza CDC ACM; si no, intenta usbserial
    cands = sorted(glob.glob("/dev/ttyACM*")) or sorted(glob.glob("/dev/ttyUSB*"))
    if not cands:
        raise RuntimeError("No se encontró /dev/ttyACM* ni /dev/ttyUSB* (¿Tiva conectada por USB-ICDI?).")
    return cands[0]

class RawTTY:
    """Pone stdin en modo raw (sin Enter) y permite lectura no bloqueante."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)  # sin canónico, sin eco
        return self
    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def read_key(self, timeout=0.02):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None

# ======= Estado compartido para el HUD =======
state = {
    "duty": None,        # porcentaje int (0..100) o None si desconocido
    "dist": None,        # cm (float/int) o None si desconocido
    "mode": "STOP",      # texto corto del último comando de movimiento
    "dirty": True,       # fuerza repintar HUD
}
state_lock = threading.Lock()

def set_state(**kwargs):
    with state_lock:
        for k, v in kwargs.items():
            state[k] = v
        state["dirty"] = True

def get_state():
    with state_lock:
        return state.copy()

def format_hud(st):
    duty = "--" if st["duty"] is None else f"{int(st['duty']):3d}%"
    dist = "--" if st["dist"] is None else f"{st['dist']:.1f} cm" if isinstance(st["dist"], float) else f"{st['dist']} cm"
    mode = st["mode"]
    return f"[SPD {duty}]  [DIST {dist:>7}]  [MODE {mode}]  (h=ayuda, q=salir)"

def print_hud(force=False, width=0):
    st = get_state()
    if not force and not st["dirty"]:
        return
    line = format_hud(st)
    if width and len(line) < width:
        line = line + " " * (width - len(line))
    sys.stdout.write("\r" + line)
    sys.stdout.flush()
    with state_lock:
        state["dirty"] = False

def reader_thread(ser: serial.Serial, stop_evt: threading.Event, console_width_holder):
    """
    Escucha la Tiva:
      - 'DUTY,xx'   → actualiza velocidad
      - 'DIST,xx.x' → actualiza distancia
    Cualquier otra línea la imprime por encima del HUD y luego repinta el HUD.
    """
    buf = b""
    while not stop_evt.is_set() and ser.is_open:
        try:
            b = ser.read(1)
            if not b:
                continue
            if b in (b"\n", b"\r"):
                if buf:
                    try:
                        line = buf.decode("utf-8", errors="ignore").strip()
                    except:
                        line = str(buf)
                    if line:
                        up = line.upper()
                        if up.startswith("DUTY,"):
                            # e.g., 'DUTY,65'
                            try:
                                val = int(up.split(",")[1].strip())
                                set_state(duty=max(0, min(100, val)))
                            except:
                                pass
                        elif up.startswith("DIST,"):
                            # e.g., 'DIST,12.3'
                            try:
                                val_str = up.split(",")[1].strip()
                                val = float(val_str) if "." in val_str else int(val_str)
                                set_state(dist=val)
                            except:
                                pass
                        else:
                            # Mensaje libre → imprimir encima del HUD
                            sys.stdout.write("\r" + " " * console_width_holder[0] + "\r")
                            sys.stdout.write(line + "\n")
                            sys.stdout.flush()
                            set_state()  # marcar HUD como sucio
                    buf = b""
            else:
                buf += b
        except Exception:
            break

def send(ser: serial.Serial, ch: str):
    ser.write(ch.encode("utf-8"))
    ser.flush()

def main():
    try:
        port = autodetect_port()
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] No se pudo abrir {port}: {e}")
        return

    print(f"[INFO] Abierto {port} @ {BAUD}")
    print("[INFO] Movimiento PERSISTE hasta STOP (SPACE/0).")
    print(HELP)

    # Ancho aproximado de consola para limpiar línea de HUD cuando haya logs
    console_width_holder = [max(80, 120)]

    stop_evt = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_evt, console_width_holder), daemon=True)
    t.start()

    # HUD inicial
    set_state(mode="STOP")
    print_hud(force=True, width=console_width_holder[0])

    last_hud = time.time()
    HUD_PERIOD = 0.1  # 10 Hz

    try:
        with RawTTY() as kb:
            while True:
                ch = kb.read_key(timeout=HUD_PERIOD)
                # repinta HUD aunque no haya tecla, a ~10 Hz
                now = time.time()
                if now - last_hud >= HUD_PERIOD:
                    print_hud(width=console_width_holder[0])
                    last_hud = now

                if not ch:
                    continue
                k = ch.lower()

                # Salir
                if k in ('\x1b', 'q', '\x03'):  # Esc, q, Ctrl+C
                    # Limpia la línea del HUD antes de salir
                    sys.stdout.write("\r" + " " * console_width_holder[0] + "\r")
                    sys.stdout.flush()
                    print("Bye!")
                    break

                # Mapeo de teclas → un solo carácter a la Tiva
                if   k == 'w':
                    send(ser, 'w'); set_state(mode="FWD")
                elif k == 's':
                    send(ser, 's'); set_state(mode="REV")
                elif k == 'a':
                    send(ser, 'a'); set_state(mode="LEFT")
                elif k == 'd':
                    send(ser, 'd'); set_state(mode="RIGHT")
                elif k == ' ':
                    send(ser, ' '); set_state(mode="STOP")
                elif k == '0':
                    send(ser, '0'); set_state(mode="STOP")
                elif k == 'k':
                    send(ser, 'k')  # duty -
                elif k == 'l':
                    send(ser, 'l')  # duty +
                elif k == 'b':
                    send(ser, 'b')  # buzzer
                elif k == 'h':
                    # Limpia HUD, imprime ayuda, y luego repinta HUD
                    sys.stdout.write("\r" + " " * console_width_holder[0] + "\r")
                    sys.stdout.flush()
                    print(HELP)
                # Otras teclas: ignorar

                # Forzar HUD tras acción
                print_hud(force=True, width=console_width_holder[0])

    except KeyboardInterrupt:
        sys.stdout.write("\r" + " " * console_width_holder[0] + "\r")
        sys.stdout.flush()
        print("Bye!")
    finally:
        stop_evt.set()
        try:
            t.join(timeout=0.5)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
