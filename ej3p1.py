#!/usr/bin/env python3
# usb_cam_colors_uart_tiva.py
# Rojo → 'w' (adelante, 50% usuario), Verde → 's' (atrás, 50% usuario), Ninguno → '0' (stop)
import cv2
import time
import argparse
import glob
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Dict
import numpy as np
import serial

# ---------------- Config cámara ----------------
@dataclass
class CameraConfig:
    index: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    fourcc: str = "MJPG"
    window_title: str = "USB Cam — Red/Green + UART Tiva (q=salir, s=foto)"
    show_fps: bool = True
    out_dir: Path = Path("./captures")

# ---------------- Detector de colores ----------------
@dataclass
class HSVRange:
    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]

class SimpleColorDetector:
    """
    Detector simple para rojo y verde en HSV.
    - Rojo: dos rangos (0–10 y 170–180).
    - Verde: 35–85 por defecto, ajustable por CLI.
    """
    def __init__(self,
                 red_ranges: Tuple[HSVRange, HSVRange] = (
                     HSVRange((0,   120, 70), (10,  255, 255)),
                     HSVRange((170, 120, 70), (180, 255, 255)),
                 ),
                 green_range: HSVRange = HSVRange((35, 80, 70), (85, 255, 255)),
                 kernel_size: int = 5):
        self.red_ranges = red_ranges
        self.green_range = green_range
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    def _mask_red(self, hsv: np.ndarray) -> np.ndarray:
        m1 = cv2.inRange(hsv, np.array(self.red_ranges[0].lower), np.array(self.red_ranges[0].upper))
        m2 = cv2.inRange(hsv, np.array(self.red_ranges[1].lower), np.array(self.red_ranges[1].upper))
        m  = cv2.bitwise_or(m1, m2)
        return self._clean_mask(m)

    def _mask_green(self, hsv: np.ndarray) -> np.ndarray:
        m = cv2.inRange(hsv, np.array(self.green_range.lower), np.array(self.green_range.upper))
        return self._clean_mask(m)

    def _clean_mask(self, m: np.ndarray) -> np.ndarray:
        # filtro morfológico simple para quitar ruido y cerrar huecos
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        return m

    def analyze(self, frame_bgr: np.ndarray) -> Dict[str, dict]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)  # ← BGR → HSV aquí
        red_mask   = self._mask_red(hsv)
        green_mask = self._mask_green(hsv)
        return {
            "red":   self._blob_info(red_mask),
            "green": self._blob_info(green_mask),
        }

    def _blob_info(self, mask: np.ndarray) -> Dict[str, Optional[object]]:
        h, w = mask.shape[:2]
        total_px = h * w
        area_px = int(cv2.countNonZero(mask))
        area_ratio = area_px / float(total_px + 1e-9)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bbox = None
        centroid = None
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            x, y, bw, bh = cv2.boundingRect(c)
            bbox = (x, y, bw, bh)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroid = (cx, cy)

        return {
            "mask": mask,
            "area_px": area_px,
            "area_ratio": area_ratio,
            "bbox": bbox,
            "centroid": centroid
        }

# ---------------- UART / Protocolo Tiva ----------------
class UARTController:
    """
    Protocolo acorde a tu Tiva (UART0IntHandler):
      - 'w' → adelante
      - 's' → atrás
      - '0' → stop
      - 'l' → subir duty un paso (inicialmente 25% → 50% usuario)
    """
    def __init__(self, baud: int = 115200, port: Optional[str] = None):
        self.baud = baud
        self.port = port or self._autodetect_port()
        if self.port is None:
            print("[WARN] No encontré /dev/ttyACM* ni /dev/ttyUSB*. UART deshabilitado.")
            self.ser = None
        else:
            print(f"[INFO] UART en {self.port} @ {baud}")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(0.3)

        self.current_state = None  # 'F','B','S'

        # Comandos de 1 caracter (más '\n' por comodidad/human-readable; la Tiva ignora '\n')
        self.cmd_forward  = "w\n"
        self.cmd_backward = "s\n"
        self.cmd_stop     = "0\n"

        # Asegurar duty = 50% usuario al iniciar (25% → 50% con una sola 'l')
        self._send_once("l\n")

    def _autodetect_port(self) -> Optional[str]:
        candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        return candidates[0] if candidates else None

    def _send_once(self, txt: str):
        if self.ser:
            try:
                self.ser.write(txt.encode("utf-8"))
            except Exception as e:
                print(f"[UART ERR] {e}")

    def set_state(self, state: str):
        """
        state in {'F','B','S'}; envía sólo si cambia el estado
        """
        if state == self.current_state:
            return
        self.current_state = state

        if state == 'F':
            self._send_once(self.cmd_forward)
            print("[UART] → FORWARD (user 50%)")
        elif state == 'B':
            self._send_once(self.cmd_backward)
            print("[UART] → BACKWARD (user 50%)")
        else:
            self._send_once(self.cmd_stop)
            print("[UART] → STOP")

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass

# ---------------- App visión + decisión → UART ----------------
class ColorToUARTApp:
    def __init__(self,
                 cfg: CameraConfig,
                 detector: SimpleColorDetector,
                 uart: UARTController,
                 area_thresh: float = 0.02,   # 2% del frame
                 hysteresis: float = 0.6,     # mantener ~0.6 s antes de confirmar cambio
                 ):
        self.cfg = cfg
        self.detector = detector
        self.uart = uart
        self.area_thresh = area_thresh
        self.hysteresis = hysteresis

        self.cap: Optional[cv2.VideoCapture] = None
        self.out_dir = cfg.out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)

        # FPS
        self._last = time.time()
        self._count = 0
        self._fps_txt = "FPS: --"

        # Histeresis temporal
        self._last_decision = 'S'
        self._last_change_t = time.time()

    def open(self) -> None:
        self.cap = cv2.VideoCapture(self.cfg.index, cv2.CAP_V4L2)
        if not self.cap or not self.cap.isOpened():
            raise SystemExit(f"No pude abrir /dev/video{self.cfg.index}.")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.cfg.fps)

        rw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        rh = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rf = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"[INFO] Cámara: {self.cfg.fourcc} {rw}x{rh} @ {rf:.1f} FPS")

        cv2.namedWindow(self.cfg.window_title, cv2.WINDOW_NORMAL)

    def run(self) -> None:
        if not self.cap:
            self.open()

        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    print("[WARN] Frame no disponible.")
                    continue

                info = self.detector.analyze(frame)
                decision = self._decide(info)

                # Histeresis temporal: confirmar cambios persistentes
                now = time.time()
                if decision != self._last_decision:
                    if (now - self._last_change_t) >= self.hysteresis:
                        self._last_decision = decision
                        self._last_change_t = now
                        self.uart.set_state(decision)
                else:
                    self._last_change_t = now

                self._draw_overlay(frame, info, decision)

                if self.cfg.show_fps:
                    self._update_fps()
                    cv2.putText(frame, self._fps_txt, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2, cv2.LINE_AA)

                cv2.imshow(self.cfg.window_title, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                elif key == ord('s'):
                    self._save_snapshot(frame)
        finally:
            # seguridad al salir
            self.uart.set_state('S')
            self.release()
            self.uart.close()

    def _decide(self, info: Dict[str, dict]) -> str:
        """
        Retorna 'F' (adelante), 'B' (atrás) o 'S' (stop)
        Prioridad cuando hay conflicto: mantener estado anterior.
        """
        red_pct = info["red"]["area_ratio"]
        gre_pct = info["green"]["area_ratio"]
        has_red = red_pct >= self.area_thresh
        has_gre = gre_pct >= self.area_thresh

        if has_red and not has_gre:
            return 'F'
        if has_gre and not has_red:
            return 'B'
        if has_red and has_gre:
            return self._last_decision   # evita oscilación si ambos aparecen
        return 'S'

    def _draw_overlay(self, frame: np.ndarray, info: Dict[str, dict], decision: str):
        h, w = frame.shape[:2]
        # Texto áreas + bbox + centroides
        for label, color, key, y in [("RED", (0,0,255), "red", 60),
                                     ("GREEN",(0,255,0),"green",100)]:
            pct = info[key]["area_ratio"] * 100.0
            txt = f"{label}: {pct:.1f}%"
            cv2.putText(frame, txt, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
            if info[key]["bbox"] is not None:
                x, yb, bw, bh = info[key]["bbox"]
                cv2.rectangle(frame, (x, yb), (x+bw, yb+bh), color, 2)
            if info[key]["centroid"] is not None:
                cx, cy = info[key]["centroid"]
                cv2.circle(frame, (cx, cy), 6, color, -1)

        # Acción actual
        msg = {"F":"FORWARD 50% (usuario)", "B":"BACKWARD 50% (usuario)", "S":"STOP"}[decision]
        cv2.putText(frame, f"ACTION: {msg}", (10, h-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2, cv2.LINE_AA)

    def _save_snapshot(self, frame):
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = self.out_dir / f"cap_{ts}.jpg"
        cv2.imwrite(str(path), frame)
        print(f"[OK] Foto guardada: {path}")

    def _update_fps(self):
        self._count += 1
        now = time.time()
        if now - self._last >= 1.0:
            fps = self._count / (now - self._last)
            self._fps_txt = f"FPS: {fps:.1f}"
            self._last = now
            self._count = 0

    def release(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

# ---------------- CLI ----------------
def build_parser():
    p = argparse.ArgumentParser(description="Rojo→'w' (adelante), Verde→'s' (atrás), Ninguno→'0' (stop), UART Tiva.")
    p.add_argument("--index",  type=int, default=0)
    p.add_argument("--width",  type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps",    type=int, default=30)
    p.add_argument("--fourcc", type=str, default="MJPG")
    p.add_argument("--area",   type=float, default=0.02, help="Umbral de área (0..1) para considerar detección.")
    p.add_argument("--hyst",   type=float, default=0.6,  help="Histeresis temporal (s) para confirmar cambios.")
    p.add_argument("--baud",   type=int,   default=115200)
    p.add_argument("--port",   type=str,   default=None, help="Forzar puerto serie, e.g. /dev/ttyACM0")
    return p

def main():
    args = build_parser().parse_args()
    cfg = CameraConfig(index=args.index, width=args.width, height=args.height,
                       fps=args.fps, fourcc=args.fourcc)

    # detector con rangos por defecto (verde 35–85). Si necesitas, puedo exponerlos por CLI.
    detector = SimpleColorDetector()

    uart = UARTController(baud=args.baud, port=args.port)

    app = ColorToUARTApp(cfg, detector, uart,
                         area_thresh=args.area,
                         hysteresis=args.hyst)
    app.run()

if __name__ == "__main__":
    main()
