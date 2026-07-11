"""
ELE495 - RPi tarafi Birlesik Sensor Bridge
Arduino'dan gelen CSV satirini okur: IR1,IR2,Distance,Heading,R,G,B,C
Thread-safe global state tutar.

Onceki heading_bridge.py'nin yerine gecer, ayni get_heading() metodunu
korur (donus_kapali_dongu.py gibi mevcut kodlar degismeden calisir),
ustune get_distance(), get_color(), get_ir() gibi ek metodlar ekler.

Kullanim:
    bridge = RobotBridge(port="/dev/ttyUSB0")
    bridge.start()
    ...
    heading = bridge.get_heading()
    distance = bridge.get_distance()
    r, g, b, c = bridge.get_color()
    ir1, ir2 = bridge.get_ir()
    ...
    bridge.request_heading_reset()
    bridge.stop()
"""

import serial
import threading
import time


class RobotBridge:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self._ser = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        self._state = {
            "ir1": None,
            "ir2": None,
            "distance": None,
            "heading": None,
            "r": None,
            "g": None,
            "b": None,
            "c": None,
            "vcc_mv": None,
            "last_update": 0.0,
            "connected": False,
            "cal_sys": None,
            "cal_gyro": None,
            "cal_accel": None,
            "cal_mag": None,
            "eeprom_yuklendi": None,
            "kayitli_sys": None,
            "kayitli_gyro": None,
            "kayitli_accel": None,
            "kayitli_mag": None,
        }

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2)  # Arduino reset'inin bitmesini bekle
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
                    print(f"[RobotBridge] Arduino uyarisi: {satir}")
                    continue

                if satir.startswith("SAVEDCAL"):
                    self._handle_saved_calibration_line(satir)
                    continue

                if satir.startswith("CAL"):
                    self._handle_calibration_line(satir)
                    continue

                self._parse_csv(satir)

            except (serial.SerialException, UnicodeDecodeError) as e:
                print(f"[RobotBridge] Seri okuma hatasi: {e}")
                with self._lock:
                    self._state["connected"] = False
                time.sleep(0.5)

    def _parse_csv(self, satir):
        # Beklenen format: IR1,IR2,Distance,Heading,R,G,B,C,VccMv
        parcalar = satir.split(",")
        if len(parcalar) != 9:
            return  # bozuk/eksik satir, atla

        try:
            ir1, ir2, distance, heading, r, g, b, c, vcc_mv = (float(p) for p in parcalar)
        except ValueError:
            return

        with self._lock:
            self._state["ir1"] = ir1
            self._state["ir2"] = ir2
            self._state["distance"] = distance
            self._state["heading"] = heading
            self._state["r"] = r
            self._state["g"] = g
            self._state["b"] = b
            self._state["c"] = c
            self._state["vcc_mv"] = vcc_mv
            self._state["last_update"] = time.time()
            self._state["connected"] = True

    def _handle_calibration_line(self, satir):
        # Format: CAL,sys,gyro,accel,mag,eeprom_yuklendi
        parcalar = satir.split(",")
        if len(parcalar) == 6:
            try:
                sys_v, gyro_v, accel_v, mag_v, yuklendi_v = (int(p) for p in parcalar[1:])
            except ValueError:
                return
            with self._lock:
                self._state["cal_sys"] = sys_v
                self._state["cal_gyro"] = gyro_v
                self._state["cal_accel"] = accel_v
                self._state["cal_mag"] = mag_v
                self._state["eeprom_yuklendi"] = bool(yuklendi_v)
            print(f"[RobotBridge] Kalibrasyon durumu -> "
                  f"sys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v} "
                  f"eeprom_yuklendi={bool(yuklendi_v)}")

    # ---------- Okuma metodlari ----------

    def get_heading(self):
        with self._lock:
            return self._state["heading"]

    def get_distance(self):
        with self._lock:
            return self._state["distance"]

    def get_vcc(self):
        """Arduino'nun 5V hattinin gercek gerilimini mV cinsinden dondurur.
        Guc yetersizligi teshisi icin kullanilabilir - dusuk deger (orn.
        4300mV altı) motorlar calisirken sensorlere yeterli guc gitmedigini
        gosterebilir."""
        with self._lock:
            return self._state["vcc_mv"]

    def get_ir(self):
        with self._lock:
            return self._state["ir1"], self._state["ir2"]

    def get_color(self):
        with self._lock:
            return (self._state["r"], self._state["g"],
                    self._state["b"], self._state["c"])

    def get_calibration(self):
        with self._lock:
            return {
                "sys": self._state["cal_sys"],
                "gyro": self._state["cal_gyro"],
                "accel": self._state["cal_accel"],
                "mag": self._state["cal_mag"],
                "eeprom_yuklendi": self._state["eeprom_yuklendi"],
            }

    def get_all(self):
        """Tum guncel state'in kopyasini dondurur (thread-safe)."""
        with self._lock:
            return dict(self._state)

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

    def _handle_saved_calibration_line(self, satir):
        # Format: SAVEDCAL,sys,gyro,accel,mag,eeprom_yuklendi
        parcalar = satir.split(",")
        if len(parcalar) == 6:
            try:
                sys_v, gyro_v, accel_v, mag_v, yuklendi_v = (int(p) for p in parcalar[1:])
            except ValueError:
                return
            with self._lock:
                self._state["kayitli_sys"] = sys_v
                self._state["kayitli_gyro"] = gyro_v
                self._state["kayitli_accel"] = accel_v
                self._state["kayitli_mag"] = mag_v
                self._state["eeprom_yuklendi"] = bool(yuklendi_v)
            print(f"[RobotBridge] KAYITLI kalibrasyon (kayit anindaki degerler) -> "
                  f"sys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v} "
                  f"eeprom_yuklendi={bool(yuklendi_v)}")

    def get_saved_calibration_info(self):
        """EEPROM'a kaydedilen kalibrasyonun, KAYIT ANINDAKI kalite seviyelerini
        dondurur (canli/su anki degerler degil). Once request_saved_calibration_info()
        cagirmalisin."""
        with self._lock:
            return {
                "sys": self._state["kayitli_sys"],
                "gyro": self._state["kayitli_gyro"],
                "accel": self._state["kayitli_accel"],
                "mag": self._state["kayitli_mag"],
                "eeprom_yuklendi": self._state["eeprom_yuklendi"],
            }

    def request_saved_calibration_info(self):
        """Arduino'dan, EEPROM'a kaydedilmis kalibrasyonun kayit anindaki
        kalite seviyelerini (sys/gyro/accel/mag) ister."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"G")

    def request_fast_mode(self):
        """Renk sensoru okumasini gecici olarak kapatir - Arduino'nun dongu
        hizini ~10-15Hz'den ~50-100Hz'e cikarir. Hassas zamanlama gerektiren
        (donus gibi) islemlerden once cagir."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"F")

    def request_normal_mode(self):
        """Renk sensoru okumasini tekrar acar (hizli moddan cikar)."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"N")

    def request_save_calibration(self):
        """Arduino'ya mevcut BNO055 kalibrasyonunu EEPROM'a kalici olarak
        kaydetmesini soyler. Sadece kalibrasyon iyiyken (orn. sys=3) cagir."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"S")

    def request_delete_calibration(self):
        """Arduino'ya kayitli kalibrasyonu silmesini soyler. Bir sonraki
        acilista Arduino sifirdan kalibrasyon isteyecektir."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(b"D")


if __name__ == "__main__":
    bridge = RobotBridge(port="/dev/ttyUSB0")
    bridge.start()
    try:
        baslangic = time.time()
        while time.time() - baslangic < 15:
            veri = bridge.get_all()
            if veri["connected"]:
                print(
                    f"Heading={veri['heading']:.1f} "
                    f"Distance={veri['distance']:.1f}cm "
                    f"IR=({veri['ir1']:.0f},{veri['ir2']:.0f}) "
                    f"RGB=({veri['r']:.0f},{veri['g']:.0f},{veri['b']:.0f}) C={veri['c']:.0f} "
                    f"Vcc={veri['vcc_mv']:.0f}mV"
                )
            time.sleep(0.2)
    finally:
        bridge.stop()
