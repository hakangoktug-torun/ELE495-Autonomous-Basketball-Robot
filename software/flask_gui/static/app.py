"""
ELE495 - Basketbol Robot Kontrol Paneli (Flask GUI) - GERCEK DONANIM SURUMU
Bu surum artik simulasyon degil - gercek RobotBridge/Arduino baglantisini
kullanir ve software/raspberry_pi/kalibrasyon_kodlari/
firlaticisiz_atis_simulasyon_test.py icindeki atis dongusunu arka planda
bir thread'de calistirir.

MIMARI NOTU: Arduino'ya seri port (/dev/ttyUSB0) uzerinden AYNI ANDA
SADECE BIR baglanti acilabilir. Bu yuzden test scriptini ayri bir terminalden
DE calistirmamalisin - bu GUI calisirken port zaten bu surec tarafindan
kullaniliyor olacak. Testi SADECE bu GUI uzerinden ("Testi Baslat" butonu)
tetikle.

Calistirma:
    cd software/flask_gui
    python3 app.py
Sonra tarayicidan: http://<rpi-ip>:5000
"""

import os
import sys
import time
import threading

from flask import Flask, jsonify, render_template, request

# ---------------------------------------------------------------------------
# software/raspberry_pi/ ve software/raspberry_pi/kalibrasyon_kodlari/
# dizinlerini arama yoluna ekle - boylece robot_bridge.py,
# donus_kapali_dongu.py ve firlaticisiz_atis_simulasyon_test.py buradan
# import edilebilir.
# ---------------------------------------------------------------------------
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_RASPI_DIR = os.path.normpath(os.path.join(_BASE_DIR, "..", "raspberry_pi"))
_KALIB_DIR = os.path.join(_RASPI_DIR, "kalibrasyon_kodlari")
sys.path.insert(0, _RASPI_DIR)
sys.path.insert(0, _KALIB_DIR)

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
import firlaticisiz_atis_simulasyon_test as atis_modulu

app = Flask(__name__)

MAKS_GECMIS_UZUNLUGU = 60


class AtisTestiKontrolcusu:
    """
    RobotBridge + motor baglantisini TEK SEFERDE acar ve GUI ile arka
    plan thread'i arasinda paylasir. Atis dongusu calisirken (aci_getir_fn
    cagrildiginda) web'den cevap gelene kadar bekler - bunu bir
    threading.Event ile yapar.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.bridge = None
        self.pwm_a = None
        self.pwm_b = None

        self.baglanti_hazir = False
        self.baglanti_hata_mesaji = None
        self.calisiyor = False
        self.score = 0
        self.gecmis = []
        self.bekleyen_girdi = None  # {"kenar_no", "bolge", "puan"} ya da None

        self._girdi_event = threading.Event()
        self._girdi_cevabi = None
        self._thread = None

    # ---------------- yardimci ----------------

    def _gecmise_ekle(self, mesaj):
        with self._lock:
            zaman_damgasi = time.strftime("%H:%M:%S")
            self.gecmis.insert(0, f"{zaman_damgasi} - {mesaj}")
            if len(self.gecmis) > MAKS_GECMIS_UZUNLUGU:
                self.gecmis.pop()
        print(f"[AtisTesti] {mesaj}")

    def _puan_ekle(self, puan):
        with self._lock:
            self.score += puan

    # ---------------- baglanti ----------------

    def baglan(self):
        if self.baglanti_hazir:
            return True

        self._gecmise_ekle("Arduino/BNO055 baglantisi kuruluyor...")
        self.bridge = RobotBridge(port=SERIAL_PORT)
        self.bridge.start()

        for _ in range(50):
            if not self.bridge.is_stale(max_age_sec=1.0):
                break
            time.sleep(0.1)

        if self.bridge.is_stale(max_age_sec=1.0):
            self.baglanti_hata_mesaji = "Sensor veri akisi yok - Arduino baglantisini kontrol et."
            self._gecmise_ekle(f"HATA: {self.baglanti_hata_mesaji}")
            self.bridge.stop()
            self.bridge = None
            return False

        self.bridge.request_fast_mode()
        time.sleep(0.1)

        self.pwm_a, self.pwm_b = motorlari_ayarla()
        self.baglanti_hazir = True
        self.baglanti_hata_mesaji = None
        self._gecmise_ekle("Baglanti hazir.")
        return True

    # ---------------- atis dongusu ----------------

    def _aci_getir(self, kenar_no, bolge_aciklama, bolge_puan):
        with self._lock:
            self.bekleyen_girdi = {
                "kenar_no": kenar_no,
                "bolge": bolge_aciklama,
                "puan": bolge_puan,
            }
        self._girdi_event.clear()
        self._gecmise_ekle(f"{kenar_no}. kenar icin atis acisi bekleniyor (GUI'den gir)...")
        self._girdi_event.wait()  # web'den cevap gelene kadar bloke olur

        with self._lock:
            cevap = self._girdi_cevabi
            self.bekleyen_girdi = None
        return cevap

    def aci_gonder(self, yon, aci):
        with self._lock:
            if self.bekleyen_girdi is None:
                return False
            if yon not in ("sol", "sag") or aci <= 0:
                return False
            self._girdi_cevabi = (yon, aci)
        self._girdi_event.set()
        return True

    def testi_baslat(self):
        with self._lock:
            if self.calisiyor:
                return False
            self.calisiyor = True

        self._thread = threading.Thread(target=self._calistir, daemon=True)
        self._thread.start()
        return True

    def _calistir(self):
        try:
            if not self.baglanti_hazir and not self.baglan():
                return

            self._gecmise_ekle("Atis simulasyonu testi basladi.")
            atis_modulu.calistir_atis_dongusu(
                self.bridge, self.pwm_a, self.pwm_b,
                aci_getir_fn=self._aci_getir,
                olay_fn=self._gecmise_ekle,
                puan_fn=self._puan_ekle,
            )
        except Exception as e:
            self._gecmise_ekle(f"HATA: Test sirasinda beklenmeyen bir sorun olustu: {e}")
        finally:
            if self.pwm_a is not None and self.pwm_b is not None:
                motorlari_durdur(self.pwm_a, self.pwm_b)
            with self._lock:
                self.calisiyor = False
                self.bekleyen_girdi = None

    def acil_durdur(self):
        if self.pwm_a is not None and self.pwm_b is not None:
            motorlari_durdur(self.pwm_a, self.pwm_b)
        # Eger o an bir acidan cevap bekleniyorsa, thread'i sonsuza kadar
        # bekletmemek icin sahte bir cevapla serbest birakiyoruz - test
        # zaten bir sonraki adimda (donus/mesafe kontrolu basarisiz olarak)
        # kendini guvenli sekilde durduracaktir.
        if self.bekleyen_girdi is not None:
            with self._lock:
                self._girdi_cevabi = ("sol", 0.1)
            self._girdi_event.set()
        self._gecmise_ekle("ACIL DURDUR tetiklendi - motorlar durduruldu.")

    def close(self):
        if self.pwm_a is not None and self.pwm_b is not None:
            motorlari_durdur(self.pwm_a, self.pwm_b)
            try:
                self.pwm_a.stop()
                self.pwm_b.stop()
                import RPi.GPIO as GPIO
                GPIO.cleanup()
            except Exception:
                pass
        if self.bridge is not None:
            self.bridge.stop()

    # ---------------- durum ----------------

    def status_dict(self):
        with self._lock:
            return {
                "baglanti_hazir": self.baglanti_hazir,
                "baglanti_hata_mesaji": self.baglanti_hata_mesaji,
                "calisiyor": self.calisiyor,
                "score": self.score,
                "bekleyen_girdi": self.bekleyen_girdi,
                "gecmis": list(self.gecmis),
            }


kontrolcu = AtisTestiKontrolcusu()


# ---------------------------------------------------------------------------
# Route'lar
# ---------------------------------------------------------------------------

@app.route("/")
def home():
    return render_template("index.html")


@app.get("/api/durum")
def durum():
    return jsonify(kontrolcu.status_dict())


@app.post("/api/atis-testi/baslat")
def atis_testi_baslat():
    basladi = kontrolcu.testi_baslat()
    sonuc = kontrolcu.status_dict()
    sonuc["baslatildi"] = basladi
    return jsonify(sonuc)


@app.post("/api/atis-testi/aci")
def atis_testi_aci():
    payload = request.get_json(silent=True) or {}
    yon = str(payload.get("yon", "")).strip().lower()
    try:
        aci = float(payload.get("aci", 0))
    except (TypeError, ValueError):
        aci = 0
    kabul_edildi = kontrolcu.aci_gonder(yon, aci)
    sonuc = kontrolcu.status_dict()
    sonuc["kabul_edildi"] = kabul_edildi
    return jsonify(sonuc)


@app.post("/api/emergency-stop")
def emergency_stop():
    kontrolcu.acil_durdur()
    return jsonify(kontrolcu.status_dict())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        kontrolcu.close()
