"""Shared helpers for the Flask robot control applications."""

import importlib
import os
import signal
import sys
import threading
import time

from flask import jsonify, render_template


MAKS_GECMIS_UZUNLUGU = 60
SURE_LIMIT_SN = 300.0
SAHA_GENISLIK_CM = 120.0
SAHA_UZUNLUK_CM = 80.0

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RASPI_DIR = os.path.normpath(os.path.join(BASE_DIR, "..", "raspberry_pi"))
KALIBRASYON_DIR = os.path.join(RASPI_DIR, "kalibrasyon_kodlari")


def raspberry_pi_yollarini_hazirla():
    """Allow legacy calibration modules to be imported without changing their paths."""
    for path in (RASPI_DIR, KALIBRASYON_DIR):
        if path not in sys.path:
            sys.path.insert(0, path)


def varsayilan_aktif_pozisyon(asama="beklemede"):
    return {
        "asama": asama,
        "pozisyon_no": None,
        "toplam_pozisyon": 0,
        "etiket": "",
        "bolge": None,
        "puan": None,
        "saha_x_cm": None,
        "saha_y_cm": None,
        "saha_genislik_cm": SAHA_GENISLIK_CM,
        "saha_uzunluk_cm": SAHA_UZUNLUK_CM,
    }


def donanim_modullerini_yukle():
    try:
        robot_bridge = importlib.import_module("robot_bridge")
        donus_kapali_dongu = importlib.import_module("donus_kapali_dongu")
        atis_modulu = importlib.import_module("ozel_navigasyon_testi_esc_sweep_2")
    except ImportError as exc:
        eksik = getattr(exc, "name", None) or str(exc)
        raise RuntimeError(
            "Donanim modulleri yuklenemedi. Raspberry Pi ortaminda "
            "requirements.txt bagimliliklarini ve RPi.GPIO/pigpio/pyserial "
            f"kurulumunu kontrol et. Eksik/hata: {eksik}"
        ) from exc

    return (
        robot_bridge.RobotBridge,
        donus_kapali_dongu.motorlari_ayarla,
        donus_kapali_dongu.motorlari_durdur,
        donus_kapali_dongu.SERIAL_PORT,
        atis_modulu,
    )


def telemetri_oku(bridge):
    if bridge is None:
        return {}

    try:
        tum_veri = bridge.get_all()
    except Exception:
        return {}

    return {
        "heading": tum_veri.get("heading"),
        "distance": tum_veri.get("distance"),
        "ir1": tum_veri.get("ir1"),
        "ir2": tum_veri.get("ir2"),
        "vcc_mv": tum_veri.get("vcc_mv"),
        "connected": tum_veri.get("connected"),
    }


class BaseAtisTestiKontrolcusu:
    """Common runtime controller for the normal and interactive ESC Flask apps."""

    test_basladi_mesaji = "Ozel navigasyon testi basladi."

    def __init__(self, skor_dinleyici_cls):
        self._lock = threading.Lock()
        self.bridge = None
        self.pwm_a = None
        self.pwm_b = None
        self._motorlari_durdur = None
        self._atis_modulu = None

        self.baglanti_hazir = False
        self.baglanti_hata_mesaji = None
        self.baslatiliyor = False
        self.calisiyor = False
        self.score = 0
        self.gecmis = []
        self.bekleyen_girdi = None
        self.aktif_pozisyon = varsayilan_aktif_pozisyon()
        self._thread = None

        self.skor_dinleyici = skor_dinleyici_cls(
            puan_fn=self._puan_ekle,
            olay_fn=self._gecmise_ekle,
        )
        self.dur_bayragi = threading.Event()
        self.SURE_LIMIT_SN = SURE_LIMIT_SN
        self.test_baslangic_zamani = None
        self._sure_asimi_tetiklendi = False

        self._sure_bekci_thread = threading.Thread(
            target=self._sure_takip_dongusu,
            daemon=True,
        )
        self._sure_bekci_thread.start()

    def rota_calisma_argumanlari(self):
        return {}

    def ek_status_alanlari(self):
        return {}

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
        with self._lock:
            self.aktif_pozisyon = dict(bilgi)

    def _sure_takip_dongusu(self):
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
                        "SURE DOLDU (5 dakika) - test otomatik olarak durduruluyor."
                    )
                    self.acil_durdur()

            time.sleep(0.5)

    def baglan(self):
        if self.baglanti_hazir:
            return True

        try:
            (
                RobotBridge,
                motorlari_ayarla,
                self._motorlari_durdur,
                SERIAL_PORT,
                self._atis_modulu,
            ) = donanim_modullerini_yukle()
        except RuntimeError as exc:
            self.baglanti_hata_mesaji = str(exc)
            self._gecmise_ekle(f"HATA: {self.baglanti_hata_mesaji}")
            return False

        try:
            self._gecmise_ekle("Arduino/BNO055 baglantisi kuruluyor...")
            self.bridge = RobotBridge(port=SERIAL_PORT)
            self.bridge.start()

            for _ in range(50):
                if not self.bridge.is_stale(max_age_sec=1.0):
                    break
                time.sleep(0.1)

            if self.bridge.is_stale(max_age_sec=1.0):
                self.baglanti_hata_mesaji = (
                    "Sensor veri akisi yok - Arduino baglantisini kontrol et."
                )
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
        except Exception as exc:
            self.baglanti_hata_mesaji = f"Donanim baglantisi kurulamadi: {exc}"
            self.baglanti_hazir = False
            self._gecmise_ekle(f"HATA: {self.baglanti_hata_mesaji}")
            if self.bridge is not None:
                try:
                    self.bridge.stop()
                except Exception:
                    pass
                self.bridge = None
            return False

    def testi_baslat(self):
        with self._lock:
            if self.calisiyor or self.baslatiliyor:
                return False
            self.baslatiliyor = True
            self.baglanti_hata_mesaji = None
            self.aktif_pozisyon = varsayilan_aktif_pozisyon("baslatiliyor")

        self.dur_bayragi.clear()

        try:
            if not self.baglanti_hazir and not self.baglan():
                with self._lock:
                    self.baslatiliyor = False
                    self.aktif_pozisyon["asama"] = "hata"
                return False
        except Exception as exc:
            with self._lock:
                self.baglanti_hata_mesaji = f"Baglanti baslatilamadi: {exc}"
                self.baslatiliyor = False
                self.aktif_pozisyon["asama"] = "hata"
            self._gecmise_ekle(f"HATA: {self.baglanti_hata_mesaji}")
            return False

        with self._lock:
            self.calisiyor = True
            self.baslatiliyor = False
            self.test_baslangic_zamani = time.time()
            self._sure_asimi_tetiklendi = False

        self._thread = threading.Thread(target=self._calistir, daemon=True)
        self._thread.start()
        return True

    def _calistir(self):
        try:
            if not self.baglanti_hazir and not self.baglan():
                return

            self._gecmise_ekle(self.test_basladi_mesaji)
            self._atis_modulu.calistir_ozel_rota_sweep(
                self.bridge,
                self.pwm_a,
                self.pwm_b,
                olay_fn=self._gecmise_ekle,
                skor_dinleyici=self.skor_dinleyici,
                dur_bayragi=self.dur_bayragi,
                durum_fn=self._pozisyon_guncelle,
                **self.rota_calisma_argumanlari(),
            )
        except Exception as exc:
            self._gecmise_ekle(
                f"HATA: Test sirasinda beklenmeyen bir sorun olustu: {exc}"
            )
        finally:
            if self.pwm_a is not None and self.pwm_b is not None:
                self._motorlari_guvenli_durdur()
            with self._lock:
                self.calisiyor = False
                self.baslatiliyor = False
                self.bekleyen_girdi = None

    def arka_plan_thread_bitmesini_bekle(self, timeout=5.0):
        thread = self._thread
        if thread is None or not thread.is_alive():
            return True

        thread.join(timeout=timeout)
        return not thread.is_alive()

    def acil_durdur(self):
        if self.pwm_a is not None and self.pwm_b is not None:
            self._motorlari_guvenli_durdur()
        self.dur_bayragi.set()
        self.skor_dinleyici.saymayi_durdur()
        self._gecmise_ekle("ACIL DURDUR tetiklendi - motorlar/ESC durduruluyor.")

    def close(self):
        if self.pwm_a is not None and self.pwm_b is not None:
            self._motorlari_guvenli_durdur()
            self._pwm_ve_gpio_temizle()
        if self.bridge is not None:
            self._bridge_guvenli_durdur()

    def _motorlari_guvenli_durdur(self):
        if self._motorlari_durdur is not None:
            try:
                self._motorlari_durdur(self.pwm_a, self.pwm_b)
            except Exception as exc:
                self._gecmise_ekle(f"UYARI: Motorlar durdurulurken hata olustu: {exc}")

    def _pwm_ve_gpio_temizle(self):
        try:
            self.pwm_a.stop()
            self.pwm_b.stop()
            import RPi.GPIO as GPIO

            GPIO.cleanup()
        except Exception as exc:
            self._gecmise_ekle(f"UYARI: GPIO temizligi tamamlanamadi: {exc}")

    def _bridge_guvenli_durdur(self):
        try:
            self.bridge.stop()
        except Exception as exc:
            self._gecmise_ekle(f"UYARI: RobotBridge durdurulamadi: {exc}")
        finally:
            self.bridge = None

    def status_dict(self):
        with self._lock:
            bridge = self.bridge
            if self.calisiyor and self.test_baslangic_zamani is not None:
                kalan_sure_sn = max(
                    0.0,
                    self.SURE_LIMIT_SN - (time.time() - self.test_baslangic_zamani),
                )
            else:
                kalan_sure_sn = self.SURE_LIMIT_SN

            durum = {
                "baglanti_hazir": self.baglanti_hazir,
                "baglanti_hata_mesaji": self.baglanti_hata_mesaji,
                "baslatiliyor": self.baslatiliyor,
                "calisiyor": self.calisiyor,
                "score": self.score,
                "gecis_sayisi": self.skor_dinleyici.gecis_sayisi(),
                "gecmis": list(self.gecmis),
                "aktif_pozisyon": dict(self.aktif_pozisyon),
                "kalan_sure_sn": kalan_sure_sn,
                "sure_limit_sn": self.SURE_LIMIT_SN,
            }
            durum.update(self.ek_status_alanlari())

        durum["telemetri"] = telemetri_oku(bridge)
        return durum


def json_durum(kontrolcu, **ek_alanlar):
    durum = kontrolcu.status_dict()
    durum.update(ek_alanlar)
    return jsonify(durum)


def ortak_route_kayitlarini_ekle(app, kontrolcu, skor_bildirimi_cevabi, request):
    @app.route("/")
    def home():
        return render_template("index.html")

    @app.get("/api/durum")
    def durum():
        return json_durum(kontrolcu)

    @app.post("/api/atis-testi/baslat")
    def atis_testi_baslat():
        return json_durum(kontrolcu, baslatildi=kontrolcu.testi_baslat())

    @app.post("/api/emergency-stop")
    def emergency_stop():
        kontrolcu.acil_durdur()
        return json_durum(kontrolcu)

    @app.route("/skor")
    def skor_route():
        return skor_bildirimi_cevabi(kontrolcu, request.args.get("sensor"))


def sigint_yakalayicisini_kur(kontrolcu, uygulama_adi):
    orijinal_sigint_handler = signal.getsignal(signal.SIGINT)

    def _sigint_yakala(sig, frame):
        print(
            f"\n[{uygulama_adi}] Ctrl+C algilandi - motorlar/ESC guvenli "
            "sekilde durduruluyor, lutfen bekleyin..."
        )
        kontrolcu.acil_durdur()

        if not kontrolcu.arka_plan_thread_bitmesini_bekle(timeout=5.0):
            print(
                f"[{uygulama_adi}] UYARI: arka plan thread'i 5 saniyede "
                "duzgunce bitmedi - yine de cikiliyor."
            )

        print(f"[{uygulama_adi}] Durdurma tamamlandi, cikiliyor.")
        signal.signal(signal.SIGINT, orijinal_sigint_handler)
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _sigint_yakala)
