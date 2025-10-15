#!/usr/bin/env python3
# usb_cam_colors_uart_tiva_bluecount_leds.py
# Rojo/Verde → UART Tiva; cuenta objetos AZULES (máx 4) y enciende 0..4 LEDs (en la Raspberry).

import cv2
import time
import argparse
import glob
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Dict, List
import numpy as np
import serial

# ===== GPIO para LEDs =====
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except Exception as e:
    print("[WARN] RPi.GPIO no disponible (¿no estás en una Raspberry o faltan permisos?). LEDs deshabilitados.")
    GPIO_AVAILABLE = False

# ---------------- Config cámara ----------------
@dataclass
class CameraConfig:
    index: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    fourcc: str = "MJPG"
    window_title: str = "USB Cam — RG + UART Tiva + Blue Count + LEDs (q=salir, s=foto)"
    show_fps: bool = True
    out_dir: Path = Path("./captures")

# ---------------- Rango HSV ----------------
@dataclass
class HSVRange:
    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]

# ---------------- Detector de colores ----------------
class SimpleColorDetector:
    """
    Rojo: 0–10 y 170–180; Verde: 35–85; Azul: 100–130 (por defecto).
    """
    def __init__(self,
                 red_ranges: Tuple[HSVRange, HSVRange] = (
                     HSVRange((0,   120, 70), (10,  255, 255)),
                     HSVRange((170, 120, 70), (180, 255, 255)),
                 ),
                 green_range: HSVRange = HSVRange((35, 80, 70), (85, 255, 255)),
                 blue_range:  HSVRange = HSVRange((100, 80, 70), (130, 255, 255)),
                 kernel_size: int = 5):
        self.red_ranges = red_ranges
        self.green_range = green_range
        self.blue_range  = blue_range
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    def _mask_red(self, hsv: np.ndarray) -> np.ndarray:
        m1 = cv2.inRange(hsv, np.array(self.red_ranges[0].lower), np.array(self.red_ranges[0].upper))
        m2 = cv2.inRange(hsv, np.array(self.red_ranges[1].lower), np.array(self.red_ranges[1].upper))
        return self._clean_mask(cv2.bitwise_or(m1, m2))

    def _mask_green(self, hsv: np.ndarray) -> np.ndarray:
        m = cv2.inRange(hsv, np.array(self.green_range.lower), np.array(self.green_range.upper))
        return self._clean_mask(m)

    def _mask_blue(self, hsv: np.ndarray) -> np.ndarray:
        m = cv2.inRange(hsv, np.array(self.blue_range.lower), np.array(self.blue_range.upper))
        return self._clean_mask(m)

    def _clean_mask(self, m: np.ndarray) -> np.ndarray:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        return m

    def _largest_blob_info(self, mask: np.ndarray) -> Dict[str, Optional[object]]:
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
                centroid = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

        return {
            "mask": mask,
            "area_px": area_px,
            "area_ratio": area_ratio,
            "bbox": bbox,
            "centroid": centroid
        }

    def _multiple_blobs(self, mask: np.ndarray, max_n: int, min_area_px: int) -> Dict[str, object]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes: List[Tuple[int,int,int,int]] = []
        cents: List[Tuple[int,int]] = []
        for c in sorted(cnts, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(c) < min_area_px:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            boxes.append((x, y, bw, bh))
            M = cv2.moments(c)
            if M["m00"] > 0:
                cents.append((int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])))
            else:
                cents.append((x + bw//2, y + bh//2))
            if len(boxes) >= max_n:
                break

        h, w = mask.shape[:2]
        total_px = h * w
        area_px_total = int(cv2.countNonZero(mask))
        area_ratio_total = area_px_total / float(total_px + 1e-9)

        return {
            "mask": mask,
            "count": len(boxes),
            "bboxes": boxes,
            "centroids": cents,
            "area_px_total": area_px_total,
            "area_ratio_total": area_ratio_total
        }

    def analyze(self, frame_bgr: np.ndarray, max_blue: int = 4, min_blue_area_ratio: float = 0.002) -> Dict[str, dict]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        red_mask   = self._mask_red(hsv)
        green_mask = self._mask_green(hsv)
        blue_mask  = self._mask_blue(hsv)

        info_red   = self._largest_blob_info(red_mask)
        info_green = self._largest_blob_info(green_mask)

        h, w = blue_mask.shape[:2]
        min_area_px = int(min_blue_area_ratio * (h*w))
        info_blue  = self._multiple_blobs(blue_mask, max_n=max_blue, min_area_px=min_area_px)

        return { "red": info_red, "green": info_green, "blue": info_blue }

# ---------------- Control de LEDs ----------------
class LEDBar:
    """
    Maneja una “barra” de 4 LEDs en pines BCM dados.
    set_count(n): enciende los primeros n LEDs (n limitado a 0..len(pins)).
    """
    def __init__(self, pins_bcm: List[int]):
        self.enabled = GPIO_AVAILABLE
        self.pins = pins_bcm
        if not self.enabled:
            print("[LED] Deshabilitado (GPIO no disponible).")
            return
        GPIO.setmode(GPIO.BCM)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
        self.set_count(0)

    def set_count(self, n: int):
        if not self.enabled: return
        n = max(0, min(n, len(self.pins)))
        for i, p in enumerate(self.pins):
            GPIO.output(p, GPIO.HIGH if i < n else GPIO.LOW)

    def close(self):
        if self.enabled:
            # apagar todos
            for p in self.pins:
                GPIO.output(p, GPIO.LOW)
            GPIO.cleanup(self.pins)

# ---------------- UART / Protocolo Tiva ----------------
class UARTController:
    """
    Tiva: 'w' adelante; 's' atrás; '0' stop; 'l' duty+ (25%→50% al iniciar).
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

        self.current_state = None
        self.cmd_forward  = "w\n"
        self.cmd_backward = "s\n"
        self.cmd_stop     = "0\n"

        # Dejar duty en 50% (usuario) al iniciar
        self._send_once("l\n")

    def _autodetect_port(self) -> Optional[str]:
        candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        return candidates[0] if candidates else None

    def _send_once(self, txt: str):
        if self.ser:
            try: self.ser.write(txt.encode("utf-8"))
            except Exception as e: print(f"[UART ERR] {e}")

    def set_state(self, state: str):
        if state == self.current_state:
            return
        self.current_state = state
        if state == 'F':
            self._send_once(self.cmd_forward);  print("[UART] → FORWARD (50% usuario)")
        elif state == 'B':
            self._send_once(self.cmd_backward); print("[UART] → BACKWARD (50% usuario)")
        else:
            self._send_once(self.cmd_stop);     print("[UART] → STOP")

    def close(self):
        if self.ser:
            try: self.ser.close()
            except: pass

# ---------------- App visión + decisión → UART + LEDs ----------------
class ColorToUARTApp:
    def __init__(self,
                 cfg: CameraConfig,
                 detector: SimpleColorDetector,
                 uart: UARTController,
                 ledbar: LEDBar,
                 area_thresh: float = 0.02,
                 hysteresis: float = 0.6,
                 max_blue: int = 4,
                 min_blue_area_ratio: float = 0.002):
        self.cfg = cfg
        self.detector = detector
        self.uart = uart
        self.ledbar = ledbar
        self.area_thresh = area_thresh
        self.hysteresis = hysteresis
        self.max_blue = max_blue
        self.min_blue_area_ratio = min_blue_area_ratio

        self.cap: Optional[cv2.VideoCapture] = None
        self.out_dir = cfg.out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self._last = time.time()
        self._count = 0
        self._fps_txt = "FPS: --"

        self._last_decision = 'S'
        self._last_change_t = time.time()
        self._last_blue_count = -1

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
                    cv2.waitKey(10)
                    continue

                info = self.detector.analyze(frame, max_blue=self.max_blue,
                                             min_blue_area_ratio=self.min_blue_area_ratio)

                # UART (rojo/verde) con histeresis
                decision = self._decide(info)
                now = time.time()
                if decision != self._last_decision:
                    if (now - self._last_change_t) >= self.hysteresis:
                        self._last_decision = decision
                        self._last_change_t = now
                        self.uart.set_state(decision)
                else:
                    self._last_change_t = now

                # LEDs según conteo de azul (0..4)
                blue_count = info["blue"]["count"]
                if blue_count != self._last_blue_count:
                    self._last_blue_count = blue_count
                    self.ledbar.set_count(blue_count)
                    print(f"[INFO] BLUE COUNT = {blue_count}/{self.max_blue}")

                # Overlay
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
            self.ledbar.set_count(0)
            self.release()
            self.uart.close()
            self.ledbar.close()

    def _decide(self, info: Dict[str, dict]) -> str:
        red_pct = info["red"]["area_ratio"]
        gre_pct = info["green"]["area_ratio"]
        has_red = red_pct >= self.area_thresh
        has_gre = gre_pct >= self.area_thresh
        if has_red and not has_gre: return 'F'
        if has_gre and not has_red: return 'B'
        if has_red and has_gre:     return self._last_decision
        return 'S'

    def _draw_overlay(self, frame: np.ndarray, info: Dict[str, dict], decision: str):
        h, w = frame.shape[:2]
        # Rojo y verde
        for label, color, key, y in [("RED", (0,0,255), "red", 60),
                                     ("GREEN",(0,255,0),"green",100)]:
            pct = info[key]["area_ratio"] * 100.0
            cv2.putText(frame, f"{label}: {pct:.1f}%", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
            if info[key]["bbox"] is not None:
                x, yb, bw, bh = info[key]["bbox"]
                cv2.rectangle(frame, (x, yb), (x+bw, yb+bh), color, 2)
            if info[key]["centroid"] is not None:
                cx, cy = info[key]["centroid"]
                cv2.circle(frame, (cx, cy), 6, color, -1)

        # Azules (máx 4) numerados
        binfo = info["blue"]
        cv2.putText(frame, f"BLUE COUNT: {binfo['count']}/4", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,0,0), 2, cv2.LINE_AA)
        for i, (box, cent) in enumerate(zip(binfo["bboxes"], binfo["centroids"]), start=1):
            x, yb, bw, bh = box
            cv2.rectangle(frame, (x, yb), (x+bw, yb+bh), (255,0,0), 2)
            cx, cy = cent
            cv2.circle(frame, (cx, cy), 6, (255,0,0), -1)
            cv2.putText(frame, str(i), (x, max(yb-5,15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2, cv2.LINE_AA)

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
    p = argparse.ArgumentParser(
        description="Rojo/Verde→UART Tiva; conteo Azul (máx 4) reflejado en 4 LEDs (GPIO).")
    p.add_argument("--index",  type=int, default=0)
    p.add_argument("--width",  type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps",    type=int, default=30)
    p.add_argument("--fourcc", type=str, default="MJPG")
    p.add_argument("--area",   type=float, default=0.02,  help="Umbral área (0..1) para rojo/verde.")
    p.add_argument("--hyst",   type=float, default=0.6,   help="Histeresis (s).")
    p.add_argument("--baud",   type=int,   default=115200)
    p.add_argument("--port",   type=str,   default=None,  help="Puerto serie, ej. /dev/ttyACM0")
    p.add_argument("--blue-max", type=int, default=4,     help="Máximo de objetos azules a reportar.")
    p.add_argument("--blue-min-area", type=float, default=0.002,
                   help="Área mínima por blob azul (fracción del frame).")
    p.add_argument("--led-pins", type=str, default="17,27,22,5",
                   help="Pines BCM para los 4 LEDs, separados por coma. Ej: 17,27,22,5")
    return p

def main():
    args = build_parser().parse_args()
    cfg = CameraConfig(index=args.index, width=args.width, height=args.height,
                       fps=args.fps, fourcc=args.fourcc)

    detector = SimpleColorDetector()
    uart = UARTController(baud=args.baud, port=args.port)

    # Parseo de pines
    pins = [int(x.strip()) for x in args.led_pins.split(",")]
    if len(pins) != 4:
        raise SystemExit("Debes especificar exactamente 4 pines BCM para --led-pins, p.ej.: 17,27,22,5")
    ledbar = LEDBar(pins)

    app = ColorToUARTApp(cfg, detector, uart, ledbar,
                         area_thresh=args.area,
                         hysteresis=args.hyst,
                         max_blue=args.blue_max,
                         min_blue_area_ratio=args.blue_min_area)
    app.run()

if __name__ == "__main__":
    main()
