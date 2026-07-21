"""
ELE495 - Cift Atici ESC Kontrolu (2 motor - dual flywheel)
Iki ayri ESC/motoru pigpio ile kontrol eder. Tek motorlu atici_esc_kontrol_pigpio.py
ile ayni mantik (kararli donanim zamanlamali sinyal), sadece iki bagimsiz
kanal icin genisletildi.

BAGLANTI:
    1. ESC sinyal -> RPi GPIO17 (fiziksel pin 11)
    2. ESC sinyal -> RPi GPIO23 (fiziksel pin 16)
    Her iki ESC'nin GND'si -> ortak GND hattina (RPi ile ayni referans)
    Her iki ESC'nin +5V/BEC teli -> RPi'ye BAGLANMAYACAK (izole birak -
    daha once konustugumuz BEC/RPi guc çakismasi sorununu onlemek icin)

KURULUM (once bunlari calistir, atici_esc_kontrol_pigpio.py'de de yapmistin):
    sudo apt update
    sudo apt install pigpio python3-pigpio
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod

Kullanim:
    python3 atici_cift_esc_kontrol.py
"""

import time
import pigpio

ESC1_PIN = 17
ESC2_PIN = 23

MIN_DARBE_US = 1000   # motor durur / minimum hiz
MAKS_DARBE_US = 2000  # tam hiz

ARM_BEKLEME_SURESI = 3.0  # saniye - ESC'lerin 'silahlanmasi' (arm) icin minimum sinyalde bekleme suresi


class EscKontrol:
    """Tek bir ESC/motor icin dusuk seviyeli kontrol (atici_esc_kontrol_pigpio.py
    ile ayni sinif, CiftEscKontrol tarafindan iki kez kullanilir)."""

    def __init__(self, pin, min_darbe_us=MIN_DARBE_US, maks_darbe_us=MAKS_DARBE_US):
        self.pin = pin
        self.min_darbe_us = min_darbe_us
        self.maks_darbe_us = maks_darbe_us
        self._pi = None
        self._hazir = False
        self._son_yuzde = 0.0

    def baglan(self, pi):
        """Paylasilan bir pigpio.pi() nesnesini kullanir (iki ESC ayni
        pigpio baglantisini paylasabilir, ayri ayri baglanti acmaya gerek yok)."""
        self._pi = pi
        self._pi.set_servo_pulsewidth(self.pin, self.min_darbe_us)
        self._hazir = True

    def hiz_ayarla(self, yuzde):
        """yuzde: 0-100 arasi hiz yuzdesi (0 = minimum/durma, 100 = tam hiz).
        Ondalikli degerler (orn. 7.1) de kullanilabilir."""
        if not self._hazir:
            raise RuntimeError("Once baglan() cagirilmali.")

        yuzde = max(0.0, min(100.0, yuzde))
        darbe_us = self.min_darbe_us + (self.maks_darbe_us - self.min_darbe_us) * (yuzde / 100.0)
        self._pi.set_servo_pulsewidth(self.pin, round(darbe_us))
        self._son_yuzde = yuzde
        return yuzde

    def durdur(self):
        if self._hazir:
            self._pi.set_servo_pulsewidth(self.pin, self.min_darbe_us)

    def sinyali_kes(self):
        if self._hazir:
            self._pi.set_servo_pulsewidth(self.pin, 0)
        self._hazir = False


class CiftEscKontrol:
    """
    Iki ESC'yi TEK pigpio baglantisi uzerinden birlikte yonetir.
    hiz_ayarla() varsayilan olarak HER IKI motoru da AYNI hiza ayarlar
    (dual flywheel'lerde genelde ikisi de ayni hizda donmesi gerekir) -
    istersen iki motoru BAGIMSIZ hizlarda da ayarlayabilirsin.
    """

    def __init__(self, pin1=ESC1_PIN, pin2=ESC2_PIN):
        self._pi = None
        self.esc1 = EscKontrol(pin1)
        self.esc2 = EscKontrol(pin2)

    def baslat(self, arm_bekle=True):
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError(
                "pigpio servisine baglanamadi - 'sudo systemctl start pigpiod' "
                "calistirdigindan emin ol."
            )

        self.esc1.baglan(self._pi)
        self.esc2.baglan(self._pi)

        if arm_bekle:
            print(f"Her iki ESC de arm ediliyor - minimum sinyalde {ARM_BEKLEME_SURESI}s "
                  f"bekleniyor (iki ESC'den de TEK, net bir 'arm oldu' tonu duymani "
                  f"bekliyoruz)...")
            time.sleep(ARM_BEKLEME_SURESI)
            print("Her iki ESC de arm edildi. Motorlar kontrole hazir.")

    def hiz_ayarla(self, yuzde, yuzde2=None):
        """
        yuzde: 1. motorun (ve yuzde2 verilmezse ikisinin BIRDEN) hizi.
        yuzde2: verilirse, 2. motor bu farkli hiza ayarlanir (bagimsiz kontrol).
        """
        u1 = self.esc1.hiz_ayarla(yuzde)
        u2 = self.esc2.hiz_ayarla(yuzde2 if yuzde2 is not None else yuzde)
        return u1, u2

    def durdur(self):
        self.esc1.durdur()
        self.esc2.durdur()

    def kapat(self):
        self.durdur()
        time.sleep(0.2)
        self.esc1.sinyali_kes()
        self.esc2.sinyali_kes()
        if self._pi is not None:
            self._pi.stop()


def main():
    cift = CiftEscKontrol()

    print("=== CIFT ATICI ESC KONTROL TESTI (pigpio) ===")
    print(f"1. ESC pin: GPIO{ESC1_PIN}, 2. ESC pin: GPIO{ESC2_PIN}")
    print(f"Darbe araligi: {MIN_DARBE_US}-{MAKS_DARBE_US}us\n")
    print("UYARI: Her iki motor/pervanenin de hareket alaninda hicbir sey/kimse olmadigindan emin ol!\n")
    input("Hazir oldugunda Enter'a bas (her iki ESC de arm edilecek)...")

    cift.baslat()

    try:
        while True:
            print("\n1) Her iki motoru AYNI hiza ayarla")
            print("2) Motorlari BAGIMSIZ hizlara ayarla")
            print("q) Cikis")
            secim = input("Secim: ").strip().lower()

            if secim == "q":
                break

            elif secim == "1":
                girdi = input("Hiz yuzdesi (0-100, ondalikli olabilir - orn. 45.5): ").strip()
                try:
                    yuzde = float(girdi)
                    if not (0 <= yuzde <= 100):
                        print("0 ile 100 arasinda bir deger gir.")
                        continue
                except ValueError:
                    print("Gecerli bir sayi gir.")
                    continue

                u1, u2 = cift.hiz_ayarla(yuzde)
                print(f"Her iki motor da %{u1:.2f} hizina ayarlandi.")

            elif secim == "2":
                try:
                    y1 = float(input("1. motor hizi (0-100): ").strip())
                    y2 = float(input("2. motor hizi (0-100): ").strip())
                    if not (0 <= y1 <= 100 and 0 <= y2 <= 100):
                        print("Degerler 0 ile 100 arasinda olmali.")
                        continue
                except ValueError:
                    print("Gecerli sayilar gir.")
                    continue

                u1, u2 = cift.hiz_ayarla(y1, y2)
                print(f"1. motor: %{u1:.2f}, 2. motor: %{u2:.2f} olarak ayarlandi.")

            else:
                print("Gecersiz secim.")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nMotorlar durduruluyor, pigpio kapatiliyor...")
        cift.kapat()


if __name__ == "__main__":
    main()
