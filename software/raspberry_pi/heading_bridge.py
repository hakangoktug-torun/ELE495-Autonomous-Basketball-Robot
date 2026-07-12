"""
ELE495 - RPi tarafi BNO055 Heading okuyucusu
Arduino'dan gelen tek satirlik heading degerini okur.
Thread-safe global state tutar, robot.py icinden cagirilabilir.

Kullanim:
    bridge = HeadingBridge(port="/dev/ttyUSB0")
    bridge.start()
    ...
    heading = bridge.get_heading()   # 0-360 derece, veya None
    ...
    bridge.request_heading_reset()   # BNO055'i yeniden kalibre etmek icin
    bridge.stop()
"""

import serial
import threading
import time


class HeadingBridge:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self._ser = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        self._state = {
            "heading": None,
            "last_update": 0.0,
            "connected": False,
            "cal_sys": None,
            "cal_gyro": None,
            "cal_accel": None,
            "cal_mag": None,
        }

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

        # Arduino, seri port acildiginda DTR tetiklemesiyle reset atar.
        time.sleep(2)
        self._ser.reset_input_buffer()

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def _read_loop(self):
        while self._running:
            try:
                satir = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not satir:
                    continue

                if satir.startswith("HATA"):
                    print(f"[HeadingBridge] Arduino uyarisi: {satir}")
                    continue

                if satir.startswith("CAL"):
                    self._handle_calibration_line(satir)
                    continue

                self._parse_heading(satir)

            except (serial.SerialException, UnicodeDecodeError) as e:
                print(f"[HeadingBridge] Seri okuma hatasi: {e}")
                with self._lock:
                    self._state["connected"] = False
                time.sleep(0.5)

    def _parse_heading(self, satir):
        try:
            heading = float(satir)
        except ValueError:
            return  # bozuk satir, atla

        with self._lock:
            self._state["heading"] = heading
            self._state["last_update"] = time.time()
            self._state["connected"] = True

    def _handle_calibration_line(self, satir):
        # Format: CAL,sys,gyro,accel,mag
        parcalar = satir.split(",")
        if len(parcalar) == 5:
            try:
                sys_v, gyro_v, accel_v, mag_v = (int(p) for p in parcalar[1:])
            except ValueError:
                return
            with self._lock:
                self._state["cal_sys"] = sys_v
                self._state["cal_gyro"] = gyro_v
                self._state["cal_accel"] = accel_v
                self._state["cal_mag"] = mag_v
            print(f"[HeadingBridge] Kalibrasyon durumu -> "
                  f"sys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v}")

    def get_calibration(self):
        """Son bilinen kalibrasyon degerlerini dondurur (thread-safe).
        Degerler None ise henuz sorgu yapilmamis demektir - once
        request_calibration_status() cagirmalisin."""
        with self._lock:
            return {
                "sys": self._state["cal_sys"],
                "gyro": self._state["cal_gyro"],
                "accel": self._state["cal_accel"],
                "mag": self._state["cal_mag"],
            }

    def get_heading(self):
        """Son okunan heading degerini dondurur (thread-safe). Yoksa None."""
        with self._lock:
            return self._state["heading"]

    def is_stale(self, max_age_sec=1.0):
        with self._lock:
            son = self._state["last_update"]
        return (time.time() - son) > max_age_sec

    def request_heading_reset(self):
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"R")

    def request_calibration_status(self):
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"C")


if __name__ == "__main__":
    bridge = HeadingBridge(port="/dev/ttyUSB0")
    bridge.start()
    try:
        baslangic = time.time()
        while time.time() - baslangic < 10:
            h = bridge.get_heading()
            if h is not None:
                print(f"Heading = {h:.1f} derece")
            time.sleep(0.1)
    finally:
        bridge.stop()
