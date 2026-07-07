"""
Donus kalibrasyonu + BNO055 ile gercek acı olcumu
Onceki donus_kalibrasyonu.py'deki motor mantigi ayni, ama artik
"gozle olcun" yerine BNO055'ten gercek donus acisini okuyup basıyoruz.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
heading_bridge.py'nin bir ust dizinde (software/raspberry_pi/) oldugunu varsayiyorum.
Farkli bir yerdeyse asagidaki sys.path satirini guncelle.
"""

import sys
import os
import time
import RPi.GPIO as GPIO

# heading_bridge.py'yi bir ust dizinden import edebilmek icin
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from heading_bridge import HeadingBridge

# ---------- Motor pinleri (mevcut kodunla ayni) ----------
IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16
HIZ = 30
SURE = 3.15  # test etmek istedigin donus suresi

# ---------- BNO055 seri port ayari ----------
SERIAL_PORT = "/dev/ttyUSB0"


def aci_farki(baslangic, bitis):
    """0-360 derece sarmalini (wrap-around) dogru hesaplayan aci farki.
    Ornek: baslangic=350, bitis=10 -> fark=20 (340 degil)."""
    fark = bitis - baslangic
    if fark < -180:
        fark += 360
    elif fark > 180:
        fark -= 360
    return fark


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
        GPIO.setup(p, GPIO.OUT)
    pwm_a = GPIO.PWM(ENA, 1000)
    pwm_b = GPIO.PWM(ENB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    bridge = HeadingBridge(port=SERIAL_PORT)
    bridge.start()

    try:
        print("BNO055 baglantisi bekleniyor...")
        for _ in range(50):  # ~5 saniye bekle
            if not bridge.is_stale(max_age_sec=1.0):
                break
            time.sleep(0.1)

        if bridge.is_stale(max_age_sec=1.0):
            print("UYARI: BNO055'ten veri gelmiyor, baglantiyi kontrol et. Yine de devam ediliyor...")

        baslangic_heading = bridge.get_heading()
        print(f"Baslangic heading: {baslangic_heading}")

        print(f"3 saniye sonra donus baslayacak. SURE={SURE}s, HIZ={HIZ}")
        time.sleep(3)

        # sol geri, sag ileri -> saat yonunde kendi ekseninde donus
        GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(HIZ)
        pwm_b.ChangeDutyCycle(HIZ)

        # Donus sirasinda canli heading yazdir
        donus_baslangic_zamani = time.time()
        while time.time() - donus_baslangic_zamani < SURE:
            h = bridge.get_heading()
            print(f"  ... donuyor, anlik heading = {h}")
            time.sleep(0.2)

        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        for p in [IN1, IN2, IN3, IN4]:
            GPIO.output(p, GPIO.LOW)

        # Motorun tam durmasi icin kisa bir bekleme (BNO055 titresim etkisini gecsin)
        time.sleep(0.3)
        bitis_heading = bridge.get_heading()

        print(f"Bitis heading: {bitis_heading}")

        if baslangic_heading is not None and bitis_heading is not None:
            fark = aci_farki(baslangic_heading, bitis_heading)
            print(f"\n=== SONUC ===")
            print(f"SURE={SURE}s icinde gercekte donulen aci: {fark:.1f} derece")
            print(f"(360 derece icin gereken tahmini sure: {SURE * 360 / abs(fark):.2f} s)" if fark != 0 else "")
        else:
            print("Heading verisi eksik, hesaplama yapilamadi.")

    except KeyboardInterrupt:
        print("Durduruldu.")
    finally:
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        for p in [IN1, IN2, IN3, IN4]:
            GPIO.output(p, GPIO.LOW)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()
        print("GPIO ve BNO055 baglantisi temizlendi.")


if __name__ == "__main__":
    main()
