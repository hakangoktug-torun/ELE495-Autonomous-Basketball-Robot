"""
ELE495 - Interaktif Rota Testi
test_surus.py'deki 'kare cizme' mantiginin ayni, ama donus acisi/yonu SABIT
(90 derece sola) DEGIL - her hedefe ulastiktan sonra SENDEN sorulur.

Senaryo:
  1) Bir cisim 30 cm'ye kadar yaklasana kadar duz git (1. hedef)
  2) Sana hangi yone, kac derece donmek istedigini sorar, o donusu yapar
  3) Bir cisim 30 cm'ye kadar yaklasana kadar duz git (2. hedef)
  4) Tekrar sorar, doner
  5) 3. hedef
  6) Tekrar sorar, doner
  7) 4. hedef
  8) Biter (4. hedeften sonra don sormuyoruz, is bitti)

Bu, DEMO icin degil - senin robotu test etmen icin.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve test_surus.py ile ayni yerde olmali)
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import (
    donus_yap, motorlari_ayarla, motorlari_durdur, aci_farki, isinma_yap,
    IN1, IN2, IN3, IN4, ENA, ENB, SERIAL_PORT
)
from test_surus import (
    ileri_git_engel_bulunca, ileri_yon_ayarla,
    ENGEL_ESIGI_CM,
)

TOPLAM_HEDEF_SAYISI = 4


def yon_ve_aci_sor(hedef_no):
    """Kullanicidan donus yonu ve acisini ister, gecerli girdi alana kadar tekrar sorar."""
    print(f"\n=== {hedef_no}. HEDEFE ULASILDI ===")
    while True:
        yon = input("Donus yonu (sol/sag): ").strip().lower()
        if yon in ("sol", "sag"):
            break
        print("Gecersiz - 'sol' ya da 'sag' yaz.")

    while True:
        aci_str = input("Donus acisi (derece, orn: 45): ").strip()
        try:
            aci = float(aci_str)
            if aci > 0:
                break
            print("Aci pozitif bir sayi olmali.")
        except ValueError:
            print("Gecerli bir sayi gir.")

    return yon, aci


def guvenli_donus_interaktif(aci, yon, bridge, pwm_a, pwm_b, maks_deneme=2):
    """test_surus.py'deki guvenli_donus ile ayni mantik - basarisizsa tekrar dener."""
    for deneme in range(1, maks_deneme + 1):
        sonuc = donus_yap(aci, yon=yon, bridge=bridge, pwm_a=pwm_a, pwm_b=pwm_b)

        if abs(sonuc) >= aci * 0.5:
            return True

        print(f"\nUYARI: Donus basarisiz gorunuyor (istenen {aci}, "
              f"gerceklesen {sonuc:.1f}). Deneme {deneme}/{maks_deneme}.")

        if deneme < maks_deneme:
            print("Tekrar deneniyor...\n")
            time.sleep(1.0)

    print(f"\nHATA: {maks_deneme} denemede de donus basarili olamadi.")
    return False


def main():
    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()

    print("BNO055/RobotBridge baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)

    if bridge.is_stale(max_age_sec=1.0):
        print("UYARI: Sensor veri akisi yok, baglantiyi kontrol et.")
        bridge.stop()
        return

    pwm_a, pwm_b = motorlari_ayarla()

    isinma_yap(bridge, pwm_a, pwm_b)

    try:
        for hedef_no in range(1, TOPLAM_HEDEF_SAYISI + 1):
            print(f"\n--- {hedef_no}. hedef araniyor ---")
            bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=ENGEL_ESIGI_CM)

            if not bulundu:
                print(f"{hedef_no}. hedef bulunamadi, test durduruldu.")
                return

            if hedef_no == TOPLAM_HEDEF_SAYISI:
                # Son hedefe ulasildi, artik donus sormuyoruz - is bitti.
                break

            yon, aci = yon_ve_aci_sor(hedef_no)

            if not guvenli_donus_interaktif(aci, yon, bridge, pwm_a, pwm_b):
                print("Donus basarisiz oldugu icin test durduruldu.")
                return

        print(f"\nTest tamamlandi - toplam {TOPLAM_HEDEF_SAYISI} hedefe ulasildi.")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
