"""
ELE495 - Basketbol Robot Kontrol Paneli (Flask GUI) - INTERAKTIF ESC SURUMU

Bu, app.py'nin FARKLI bir versiyonudur. SWEEP MANTIGI (2-gecis otomatik
bitirme, sweep acisi adimlari, skor hesaplama vb.) HIC DEGISTIRILMEDI -
ozel_navigasyon_testi_esc_sweep_2.py AYNEN kullaniliyor.

TEK FARK: ESC hizi artik POZISYONLAR listesindeki sabit degerlerden
OTOMATIK alinmiyor - robot her atis pozisyonuna vardiginda GUI SENDEN
ESC hizini SORUYOR (test BEKLEMEDE kalir, ESC 0'da/kapali durur), sen
degeri girip gonderdikten sonra ESC o hizda calismaya baslar. Ayrica o
pozisyonun butun sweep suresi boyunca (10s'lik beklemeler sirasinda da)
istedigin an yeni bir deger gonderip ESC hizini CANLI degistirebilirsin.

Diger tum ozellikler (Acil Durdur, break-beam skor entegrasyonu, Ctrl+C
guvenligi) app.py ile AYNI.

MIMARI NOTU: Arduino'ya seri port (/dev/ttyUSB0) uzerinden AYNI ANDA
SADECE BIR baglanti acilabilir. app.py VE bu dosyayi AYNI ANDA calistirma -
port cakismasi olur. Hangisini kullanacaksan sadece onu calistir.

Calistirma:
    cd software/flask_gui
    python3 app_esc_interaktif.py
Sonra tarayicidan: http://<rpi-ip>:5000
"""

import os
import sys
import time
import signal
import threading

from flask import Flask, jsonify, render_template, request

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_RASPI_DIR = os.path.normpath(os.path.join(_BASE_DIR, "..", "raspberry_pi"))
_KALIB_DIR = os.path.join(_RASPI_DIR, "kalibrasyon_kodlari")
sys.path.insert(0, _RASPI_DIR)
sys.path.insert(0, _KALIB_DIR)

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
from skor_dinleyici import SkorDinleyici
from esc_hiz_kontrolcusu import EscHizKontrolcusu
import ozel_navigasyon_testi_esc_sweep_2 as atis_modulu

app = Flask(__name__)

MAKS_GECMIS_UZUNLUGU = 60


class AtisTestiKontrolcusu:
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
        self.bekleyen_girdi = None

        # YENI: robotun su anki pozisyonunu/bolgesini genel loglardan AYRI,
        # yapilandirilmis bicimde tutan durum - GUI'de ozel bir panelde
        # gosterilir. calistir_ozel_rota_sweep'in durum_fn callback'i ile
        # guncellenir (bkz. _pozisyon_guncelle).
        self.aktif_pozisyon = {
            "asama": "beklemede",
            "pozisyon_no": None,
            "toplam_pozisyon": 0,
            "etiket": "",
            "bolge": None,
            "puan": None,
        }

        self._thread = None

        self.skor_dinleyici = SkorDinleyici(
            puan_fn=self._puan_ekle, olay_fn=self._gecmise_ekle
        )

        # YENI: ESC hizini GUI'den sormak/canli guncellemek icin paylasilan
        # kontrolcu. Sweep mantigina hic dokunulmadi - sadece esc_hiz
        # kaynagi degisti (sabit deger yerine bu nesne).
        self.esc_hiz_kontrolcusu = EscHizKontrolcusu(olay_fn=self._gecmise_ekle)

        self.dur_bayragi = threading.Event()

        # YENI: 5 dakikalik demo suresi sayaci - bkz. app.py'deki ayni
        # mekanizma icin detayli yorumlar.
        self.SURE_LIMIT_SN = 300.0  # 5 dakika
        self.test_baslangic_zamani = None
        self._sure_asimi_tetiklendi = False

        self._sure_bekci_thread = threading.Thread(
            target=self._sure_takip_dongusu, daemon=True
        )
        self._sure_bekci_thread.start()

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

    def _pozisyon_guncelle(self, bilgi):
        """
        ozel_navigasyon_testi_esc_sweep_2.py'nin durum_fn callback'i - her
        onemli asama gecisinde (yeni pozisyona varis, gecise baslama,
        bitis, hata, durdurma) cagrilir.
        """
        with self._lock:
            self.aktif_pozisyon = dict(bilgi)

    def _sure_takip_dongusu(self):
        """
        Sürekli (0.5s aralikla) calisan bekci dongusu - test calisirken
        gecen sureyi kontrol edip SURE_LIMIT_SN (5 dakika) dolunca
        otomatik Acil Durdur tetikler. Ayri bir daemon thread'de calisir,
        ana test thread'inden BAGIMSIZDIR.
        """
        while True:
            with self._lock:
                calisiyor = self.calisiyor
                baslangic = self.test_baslangic_zamani
                tetiklendi = self._sure_asimi_tetiklendi

            if calisiyor and baslangic is not None and not tetiklendi:
                gecen = time.time() - baslangic
                if gecen >= self.SURE_LIMIT_SN:
                    with self._lock:
                        self._sure_asimi_tetiklendi = True
                    self._gecmise_ekle(
                        f"SURE DOLDU (5 dakika) - test otomatik olarak durduruluyor."
                    )
                    self.acil_durdur()

            time.sleep(0.5)

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

    # ---------------- test dongusu ----------------

    def testi_baslat(self):
        with self._lock:
            if self.calisiyor:
                return False
            self.calisiyor = True

        self.dur_bayragi.clear()

        with self._lock:
            self.aktif_pozisyon = {
                "asama": "baslatiliyor",
                "pozisyon_no": None,
                "toplam_pozisyon": 0,
                "etiket": "",
                "bolge": None,
                "puan": None,
            }

        # YENI: 5 dakikalik sayaci baslat (sifirla).
        with self._lock:
            self.test_baslangic_zamani = time.time()
            self._sure_asimi_tetiklendi = False

        self._thread = threading.Thread(target=self._calistir, daemon=True)
        self._thread.start()
        return True

    def _calistir(self):
        try:
            if not self.baglanti_hazir and not self.baglan():
                return

            self._gecmise_ekle("Ozel navigasyon testi (interaktif ESC) basladi.")
            atis_modulu.calistir_ozel_rota_sweep(
                self.bridge, self.pwm_a, self.pwm_b,
                olay_fn=self._gecmise_ekle,
                skor_dinleyici=self.skor_dinleyici,
                dur_bayragi=self.dur_bayragi,
                esc_hiz_kontrolcusu=self.esc_hiz_kontrolcusu,
                durum_fn=self._pozisyon_guncelle,
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
        self.dur_bayragi.set()
        self.skor_dinleyici.saymayi_durdur()
        self._gecmise_ekle("ACIL DURDUR tetiklendi - motorlar/ESC durduruluyor.")

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
            if self.calisiyor and self.test_baslangic_zamani is not None:
                kalan_sure_sn = max(0.0, self.SURE_LIMIT_SN - (time.time() - self.test_baslangic_zamani))
            else:
                kalan_sure_sn = self.SURE_LIMIT_SN

            return {
                "baglanti_hazir": self.baglanti_hazir,
                "baglanti_hata_mesaji": self.baglanti_hata_mesaji,
                "calisiyor": self.calisiyor,
                "score": self.score,
                "gecis_sayisi": self.skor_dinleyici.gecis_sayisi(),
                "gecmis": list(self.gecmis),
                "esc": self.esc_hiz_kontrolcusu.durum(),
                "aktif_pozisyon": dict(self.aktif_pozisyon),
                "kalan_sure_sn": kalan_sure_sn,
                "sure_limit_sn": self.SURE_LIMIT_SN,
            }


kontrolcu = AtisTestiKontrolcusu()


# ---------------------------------------------------------------------------
# SIGINT (Ctrl+C) yakalayicisi - bkz. app.py'deki ayni mekanizma
# ---------------------------------------------------------------------------
_orijinal_sigint_handler = signal.getsignal(signal.SIGINT)


def _sigint_yakala(sig, frame):
    print("\n[app_esc_interaktif.py] Ctrl+C algilandi - motorlar/ESC guvenli "
          "sekilde durduruluyor, lutfen bekleyin...")
    kontrolcu.acil_durdur()

    thread = kontrolcu._thread
    if thread is not None and thread.is_alive():
        thread.join(timeout=5.0)
        if thread.is_alive():
            print("[app_esc_interaktif.py] UYARI: arka plan thread'i 5 saniyede "
                  "duzgunce bitmedi - yine de cikiliyor.")

    print("[app_esc_interaktif.py] Durdurma tamamlandi, cikiliyor.")
    signal.signal(signal.SIGINT, _orijinal_sigint_handler)
    raise KeyboardInterrupt()


signal.signal(signal.SIGINT, _sigint_yakala)


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


@app.post("/api/emergency-stop")
def emergency_stop():
    kontrolcu.acil_durdur()
    return jsonify(kontrolcu.status_dict())


@app.post("/api/esc-hiz")
def esc_hiz_ayarla():
    """
    YENI ROUTE: GUI'den ESC hizi gonderilir. Body: {"hiz": 12.4}

    - Eger o an bir pozisyon ilk ESC hizini BEKLIYORSA, bu deger o
      bekleyisi tamamlar ve ESC o hizda calismaya baslar.
    - Eger sweep zaten calisiyorsa (bekleme dongusu icindeyse), bu deger
      CANLI olarak uygulanir (en gec ~0.5s icinde).
    """
    payload = request.get_json(silent=True) or {}
    try:
        hiz = float(payload.get("hiz"))
    except (TypeError, ValueError):
        return jsonify({"hata": "Gecerli bir 'hiz' sayisi gonder."}), 400

    kontrolcu.esc_hiz_kontrolcusu.hiz_ayarla(hiz)
    return jsonify(kontrolcu.status_dict())


@app.route("/skor")
def skor_route():
    """Arduino R4 WiFi'nin break-beam sensorlerinden gonderdigi bildirimi
    karsilar - bkz. app.py'deki ayni route."""
    sensor_no = request.args.get("sensor")
    kontrolcu.skor_dinleyici.gecis_bildir(sensor_no)
    return "OK"


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        kontrolcu.close()
