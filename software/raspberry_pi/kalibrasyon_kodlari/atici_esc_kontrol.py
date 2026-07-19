"""
ELE495 - Atici ESC Kontrolu - PIGPIO SURUMU (kararli donanim zamanlamali sinyal)
RPi.GPIO'nun yazilimsal PWM'i, ESC'nin arm asamasinda kararsiz/gecersiz
sinyal olarak algiladigi bir davranisa yol aciyordu (surekli/artan bip sesi
- ESC'nin 'sinyal kararsiz' uyarisi). pigpio, DMA tabanli calisip darbe
genisligini mikrosaniye hassasiyetinde, CPU yukunden bagimsiz urettigi icin
bu sorunu cozmesi beklenir.

KURULUM (once bunlari calistir):
    sudo apt update
    sudo apt install pigpio python3-pigpio
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod

(pigpiod, arka planda surekli calisan bir servis - sistem her acildiginda
otomatik baslamasi icin 'enable' kismini bir kez calistirman yeterli.)

Kullanim:
    python3 atici_esc_kontrol_pigpio.py
"""

import time
import pigpio

ESC_PIN = 17

MIN_DARBE_US = 1000   # motor durur / minimum hiz
MAKS_DARBE_US = 2000  # tam hiz

ARM_BEKLEME_SURESI = 3.0  # saniye - ESC'nin 'silahlanmasi' (arm) icin minimum sinyalde bekleme suresi


class EscKontrol:
    def __init__(self, pin=ESC_PIN, min_darbe_us=MIN_DARBE_US, maks_darbe_us=MAKS_DARBE_US):
        self.pin = pin
        self.min_darbe_us = min_darbe_us
        self.maks_darbe_us = maks_darbe_us
        self._pi = None
        self._hazir = False

    def baslat(self, arm_bekle=True):
        """pigpio baglantisini kurar ve ESC'yi 'arm' eder (minimum sinyalde
        bir sure bekleyerek). Motoru calistirmadan once BUNU cagirmalisin."""
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError(
                "pigpio servisine baglanamadi - 'sudo systemctl start pigpiod' "
                "calistirdigindan emin ol."
            )

        self._pi.set_servo_pulsewidth(self.pin, self.min_darbe_us)
        self._hazir = True

        if arm_bekle:
            print(f"ESC arm ediliyor - minimum sinyalde {ARM_BEKLEME_SURESI}s bekleniyor "
                  f"(ESC'nin bip sesini dinle - artik TEK, net bir 'arm oldu' tonu "
                  f"duymani bekliyoruz, oncekindeki gibi surekli/artan bip degil)...")
            time.sleep(ARM_BEKLEME_SURESI)
            print("ESC arm edildi (varsayilan). Motor kontrole hazir.")

    def hiz_ayarla(self, yuzde):
        """yuzde: 0-100 arasi hiz yuzdesi (0 = minimum/durma sinyali, 100 = tam hiz)."""
        if not self._hazir:
            raise RuntimeError("Once baslat() cagirmalisin.")

        yuzde = max(0.0, min(100.0, yuzde))
        darbe_us = self.min_darbe_us + (self.maks_darbe_us - self.min_darbe_us) * (yuzde / 100.0)
        self._pi.set_servo_pulsewidth(self.pin, darbe_us)

    def durdur(self):
        """Motoru durma sinyaline (minimum darbe) getirir."""
        if self._hazir:
            self._pi.set_servo_pulsewidth(self.pin, self.min_darbe_us)

    def kapat(self):
        """Sinyali tamamen keser (pulsewidth=0, servo/ESC sinyali birakir) ve
        pigpio baglantisini kapatir."""
        if self._pi is not None:
            self.durdur()
            time.sleep(0.2)
            self._pi.set_servo_pulsewidth(self.pin, 0)  # sinyali tamamen kes
            self._pi.stop()
        self._hazir = False


def main():
    esc = EscKontrol(pin=ESC_PIN)

    print("=== ATICI ESC KONTROL TESTI (pigpio) ===")
    print(f"Pin: GPIO{ESC_PIN}, Darbe araligi: {MIN_DARBE_US}-{MAKS_DARBE_US}us\n")
    print("UYARI: Motor/pervane hareket alaninda hicbir sey/kimse olmadigindan emin ol!\n")
    input("Hazir oldugunda Enter'a bas (ESC arm edilecek)...")

    esc.baslat()

    try:
        while True:
            girdi = input("\nHiz yuzdesi (0-100), 'q' ile cikis: ").strip().lower()
            if girdi == "q":
                break
            try:
                yuzde = float(girdi)
                if not (0 <= yuzde <= 100):
                    print("0 ile 100 arasinda bir deger gir.")
                    continue
            except ValueError:
                print("Gecerli bir sayi gir.")
                continue

            esc.hiz_ayarla(yuzde)
            print(f"Hiz %{yuzde:.0f} olarak ayarlandi.")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nMotor durduruluyor, GPIO temizleniyor...")
        esc.kapat()


if __name__ == "__main__":
    main()
