# rasp_turn_console.py
import sys, threading, tty, termios, time
import serial

PORT = "/dev/ttyUSB0"   # <-- cámbialo si usas /dev/ttyACM0, etc.
BAUD = 115200

turn_mode = threading.Event()   # True cuando Tiva pide el giro

def reader(ser):
    """Lee y muestra la salida de la Tiva. Detecta el prompt TURN?"""
    buf = b""
    while True:
        data = ser.read(256)
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                s = line.decode("utf-8", errors="ignore").strip()
            except:
                s = str(line)
            print("\r" + s + "\n> ", end="", flush=True)
            if s.startswith("TURN?"):
                turn_mode.set()

def getch():
    """Lee una tecla (no bloqueante); devuelve '' si no hay."""
    import fcntl, os
    fd = sys.stdin.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    try:
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        ch = sys.stdin.read(1)
        return ch
    except Exception:
        return ""
    finally:
        fcntl.fcntl(fd, fcntl.F_SETFL, fl)

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print("Conectado a", PORT)
    print("Controles: w/s/a/d, espacio=stop, k/l=velocidad, b=buzzer")
    print("Cuando aparezca 'TURN?' escribe L/R + grados y ENTER (ej: L90).")
    print("> ", end="", flush=True)

    # hilo lector
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    # ponemos stdin en modo raw para teclas instantáneas
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while True:
            if turn_mode.is_set():
                # Volver temporalmente a modo “cooked” para usar input()
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                try:
                    cmd = input("Giro (L/R + grados, ej L90): ").strip().upper()
                except EOFError:
                    cmd = ""
                # Validación sencilla
                if len(cmd) >= 2 and cmd[0] in ("L","R") and cmd[1:].isdigit():
                    deg = int(cmd[1:])
                    if deg < 1: deg = 90
                    if deg > 360: deg = 360
                    payload = f"{cmd[0]}{deg}\n".encode()
                    ser.write(payload)
                    print("Enviado:", payload.decode().strip())
                else:
                    # permitir solo lado si quieres que la Tiva use 90 por defecto
                    if cmd in ("L","R"):
                        ser.write((cmd + "90\n").encode())
                        print("Enviado:", cmd + "90")
                    else:
                        print("Entrada inválida. Cancelo (ESPACIO si quieres parar).")
                        ser.write(b" \n")  # espacio = cancelar/stop en tu firmware
                turn_mode.clear()
                # Volver a raw
                tty.setcbreak(fd)
                print("> ", end="", flush=True)
                continue

            # modo normal: teclas instantáneas
            ch = getch()
            if not ch:
                time.sleep(0.01)
                continue

            # mapa básico (se envía tal cual a la Tiva)
            if ch in "wsadklb0":
                ser.write(ch.encode())
            elif ch == " ":
                ser.write(b" ")
            elif ch in ("\x03", "\x04", "q"):  # Ctrl-C / Ctrl-D / q para salir
                print("\nSaliendo…")
                break
            # ignora otras teclas
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        ser.close()

if __name__ == "__main__":
    main()