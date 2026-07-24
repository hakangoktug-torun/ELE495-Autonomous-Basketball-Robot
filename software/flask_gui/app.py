"""
ELE495 - Basketbol Robot Kontrol Paneli (Flask GUI) - GERCEK DONANIM SURUMU
Bu surum ozel_navigasyon_testi_esc_sweep_2.py icindeki sabit-aci +
otomatik-sweep atis dongusunu arka planda bir thread'de calistirir.

GUNCELLEMELER (bu surum):
  1) SKOR DINLEYICI: Break-beam sensorlerinden (Arduino R4 WiFi -> /skor
     HTTP GET) gelen top gecisi bildirimleri SkorDinleyici uzerinden
     sweep dongusune aktarilir. Bir pozisyonda 2 basarili gecis olunca
     sweep otomatik durur ve GUI'deki skor, pozisyonun SABIT puanina
     gore (kirmizi=3, yesil=2) aninda artar.
  2) ACIL DURDUR (GERCEKTEN CALISAN VERSIYON): "Acil Durdur" butonuna
     basildiginda bir threading.Event (dur_bayragi) set edilir,
     ozel_navigasyon_testi_esc_sweep_2.py icindeki TUM hareket
     fonksiyonlari bunu kontrol edip ANINDA durur ve TestDurduruldu
     istisnasiyla butun rota guvenli sekilde sonlandirilir (ESC de dahil
     - esc.kapat() her zaman calisir).
  3) CTRL+C (SIGINT) YAKALAYICI: arka plandaki test thread'i bir DAEMON
     thread oldugu icin, surec aniden kapansa esc.kapat() FINALLY blogu
     hic calismadan thread oldurulebilirdi. ESC kontrolu buyuk ihtimalle
     pigpio kullaniyor - pigpio, sinyali surdurmek icin ayri/kalici bir
     daemon (pigpiod) kullandigi icin, Python sureci kapansa bile ESC'ye
     giden sinyal KESILMEYEBILIR. Bu yuzden bir SIGINT yakalayicisi
     ekleniyor: Ctrl+C'ye basildiginda once dur_bayragi set edilip arka
     plan thread'inin (ve onun esc.kapat() cagrisinin) duzgunce bitmesi
     bekleniyor, ANCAK ONDAN SONRA surecin kapanmasina izin veriliyor.

MIMARI NOTU: Arduino'ya seri port (/dev/ttyUSB0) uzerinden AYNI ANDA
SADECE BIR baglanti acilabilir. Bu yuzden test scriptini ayri bir terminalden
DE calistirmamalisin - bu GUI calisirken port zaten bu surec tarafindan
kullaniliyor olacak. Testi SADECE bu GUI uzerinden ("Testi Baslat" butonu)
tetikle. Ayrica app_esc_interaktif.py ile AYNI ANDA calistirma - port
cakismasi olur.

NOT: /skor route'u, Arduino R4 WiFi (break-beam sensor kontrolcusu) ile
AYNI Flask sunucusu uzerinden (bu app.py, port 5000) calisir - ayri bir
sunucuya gerek yok. Arduino kodundaki RPI_IP ve RPI_PORT bu makinenin
IP'sine ve 5000'e isaret etmeli.

Calistirma:
    cd software/flask_gui
    python3 app.py
Sonra tarayicidan: http://<rpi-ip>:5000
"""

import os
import sys
import time
import signal
import threading

from flask import Flask, jsonify, render_template, request

# ---------------------------------------------------------------------------
# software/raspberry_pi/ ve software/raspberry_pi/kalibrasyon_kodlari/
# dizinlerini arama yoluna ekle - boylece robot_bridge.py,
# donus_kapali_dongu.py, skor_dinleyici.py ve
# ozel_navigasyon_testi_esc_sweep_2.py buradan import edilebilir.
# ---------------------------------------------------------------------------
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_RASPI_DIR = os.path.normpath(os.path.join(_BASE_DIR, "..", "raspberry_pi"))
_KALIB_DIR = os.path.join(_RASPI_DIR, "kalibrasyon_kodlari")
sys.path.insert(0, _RASPI_DIR)
sys.path.insert(0, _KALIB_DIR)

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
from skor_dinleyici import SkorDinleyici
import ozel_navigasyon_testi_esc_sweep_2 as atis_modulu

app = Flask(__name__)

MAKS_GECMIS_UZUNLUGU = 60


class AtisTestiKontrolcusu:
    """
    RobotBridge + motor baglantisini TEK SEFERDE acar ve GUI ile arka
    plan thread'i arasinda paylasir. skor_dinleyici, break-beam
    sensorlerinden gelen top gecisi bildirimlerini sweep dongusune aktarir.
    dur_bayragi, Acil Durdur/Ctrl+C sinyalini arka plan thread'ine iletir.
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
        self.bekleyen_girdi = None

        # Robotun su anki pozisyonunu/bolgesini genel loglardan AYRI,
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

        # Break-beam sensorlerinden (Arduino /skor) gelen top gecisi
        # bildirimlerini sweep dongusune aktaran paylasilan nesne.
        self.skor_dinleyici = SkorDinleyici(
            puan_fn=self._puan_ekle, olay_fn=self._gecmise_ekle
        )

        # Acil Durdur / Ctrl+C sinyalini arka plan thread'ine iletmek icin
        # kullanilan bayrak. testi_baslat() cagrilinca temizlenir (clear),
        # acil_durdur() ya da SIGINT yakalayicisi tarafindan set edilir.
        self.dur_bayragi = threading.Event()

        # YENI: 5 dakikalik demo suresi sayaci. Test basladiginda
        # test_baslangic_zamani kaydedilir, ayri bir bekci thread'i
        # (bkz. __init__ sonu ve _sure_takip_dongusu) surekli suresi
        # dolup dolmadigini kontrol edip dolunca otomatik Acil Durdur
        # tetikler. GUI, status_dict()'teki kalan_sure_sn alaniyla
        # geri sayimi gosterir.
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
        bitis, hata, durdurma) cagrilir. self.aktif_pozisyon'u gunceller,
        bu da /api/durum uzerinden GUI'ye ayri bir alan olarak gider.
        """
        with self._lock:
            self.aktif_pozisyon = dict(bilgi)

    def _sure_takip_dongusu(self):
        """
        Sürekli (0.5s aralikla) calisan bekci dongusu - test calisirken
        gecen sureyi kontrol edip SURE_LIMIT_SN (5 dakika) dolunca
        otomatik Acil Durdur tetikler. Ayri bir daemon thread'de calisir,
        ana test thread'inden BAGIMSIZDIR - boylece test thread'i ne
        yaparsa yapsin (hangi hareket fonksiyonunda olursa olsun) sure
        kontrolu asla atlanmaz.
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

        # Onceki bir acil durdurmadan kalmis olabilecek bayragi temizle -
        # yoksa yeni test daha baslamadan ANINDA iptal edilir.
        self.dur_bayragi.clear()

        # Yeni test icin pozisyon durumunu sifirla.
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

            self._gecmise_ekle("Ozel navigasyon testi (sweep) basladi.")
            atis_modulu.calistir_ozel_rota_sweep(
                self.bridge, self.pwm_a, self.pwm_b,
                olay_fn=self._gecmise_ekle,
                skor_dinleyici=self.skor_dinleyici,
                dur_bayragi=self.dur_bayragi,
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
        """
        GERCEKTEN CALISAN Acil Durdur: hem ANINDA motorlari durdurur (bu
        satir aninda etkili olur) HEM DE dur_bayragi'ni set ederek arka
        planda calisan test thread'ine "hemen dur" sinyali gonderir - bu
        sinyal, o an calismakta olan HERHANGI bir donus/ileri-gitme/sweep
        bekleme adiminda en gec bir sonraki kontrol turunde (~20-50ms)
        yakalanip tum rotayi guvenli sekilde sonlandirir (ESC dahil).
        """
        if self.pwm_a is not None and self.pwm_b is not None:
            motorlari_durdur(self.pwm_a, self.pwm_b)
        self.dur_bayragi.set()
        # Sayimi da durduruyoruz - acil durdurdan sonra gelebilecek
        # gecikmeli sensor tetiklemeleri yanlislikla puan eklemesin.
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
                "aktif_pozisyon": dict(self.aktif_pozisyon),
                "kalan_sure_sn": kalan_sure_sn,
                "sure_limit_sn": self.SURE_LIMIT_SN,
            }


kontrolcu = AtisTestiKontrolcusu()


# ---------------------------------------------------------------------------
# SIGINT (Ctrl+C) yakalayicisi - ESC'nin (pigpio uzerinden) surec kapansa
# bile aktif kalma riskine karsi, kapanmadan once arka plan thread'ine
# duzgunce durma ve temizlenme (esc.kapat()) sansi tanir.
# ---------------------------------------------------------------------------
_orijinal_sigint_handler = signal.getsignal(signal.SIGINT)


def _sigint_yakala(sig, frame):
    print("\n[app.py] Ctrl+C algilandi - motorlar/ESC guvenli sekilde durduruluyor, "
          "lutfen bekleyin...")
    kontrolcu.acil_durdur()

    thread = kontrolcu._thread
    if thread is not None and thread.is_alive():
        thread.join(timeout=5.0)
        if thread.is_alive():
            print("[app.py] UYARI: arka plan thread'i 5 saniyede duzgunce "
                  "bitmedi - yine de cikiliyor (ESC/motor durumu belirsiz "
                  "olabilir, elle kontrol edin).")

    print("[app.py] Durdurma tamamlandi, cikiliyor.")
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


@app.route("/skor")
def skor_route():
    """
    Arduino R4 WiFi'nin break-beam sensorlerinden gonderdigi bildirimi
    karsilar: GET /skor?sensor=N

    Sayim o an aktif bir atis pozisyonu icin baslatilmissa (yani robot
    bir atis pozisyonunda sweep yaparken, ESC gercekten donuyorken) gecis
    sayilir ve GUI'deki skor pozisyonun sabit puanina gore (kirmizi=3,
    yesil=2) ANINDA artar. Sayim aktif degilse (orn. pozisyonlar arasi
    hareket sirasinda, ya da ESC henuz calismiyorken gelen bir tetikleme)
    sessizce yoksayilir.
    """
    sensor_no = request.args.get("sensor")
    kontrolcu.skor_dinleyici.gecis_bildir(sensor_no)
    return "OK"


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        kontrolcu.close()
