"""
ELE495 - Ozel Rota Testi
Sabit bir senaryo dener (kare degil, ozel bir yol):

  1) 40 derece SAGA don
  2) 0.5 saniye duz git (zaman bazli)
  3) 50 derece SAGA don
  4) Onundeki hedefe 65 cm kalana kadar duz git (mesafe sensoruyle)
  5) 75 derece SOLA don
  6) 75 derece SAGA don
  7) Onundeki hedefe 40 cm kalana kadar duz git (mesafe sensoruyle)
  8) 85 derece SOLA don
  9) 85 derece SAGA don
  10) Onundeki hedefe 20 cm kalana kadar duz git (mesafe sensoruyle)
  11) 100 derece SOLA don
  12) 10 derece SAGA don
  13) 2 saniye duz git (zaman bazli)
  14) 25 derece SOLA don
  15) Bitir

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
    motorlari_ayarla, motorlari_durdur, aci_farki, SERIAL_PORT
)
from test_surus import (
    guvenli_donus, ileri_git_engel_bulunca, ileri_yon_ayarla,
    SOL_HIZ, SAG_HIZ, DUZELTME_KAZANCI, DUZELTME_KAZANCI_I, MAKS_DUZELTME,
    MAKS_INTEGRAL,
)


def ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure_saniye):
    """
    Belirtilen sure boyunca duz ileri gider (mesafe sensoru KULLANILMAZ,
    sadece zaman). BNO055 heading feedback ile saga/sola kaymayi (drift)
    anlik olarak duzeltir - test_surus.py'deki ileri_git_engel_bulunca'nin
    ayni PI mantigi, ama durma kosulu mesafe degil SURE.
    """
    print(f"{sure_saniye} saniye duz gidiliyor (zaman bazli)...")

    ileri_yon_ayarla()

    hedef_heading = bridge.get_heading()
    temel_sol, temel_sag = SOL_HIZ, SAG_HIZ
    pwm_a.ChangeDutyCycle(temel_sol)
    pwm_b.ChangeDutyCycle(temel_sag)

    integral = 0.0
    son_zaman = time.time()
    baslangic = time.time()

    while time.time() - baslangic < sure_saniye:
        simdiki_heading = bridge.get_heading()
        simdi = time.time()
        dt = simdi - son_zaman
        son_zaman = simdi

        if simdiki_heading is not None and hedef_heading is not None:
            hata = aci_farki(hedef_heading, simdiki_heading)

            integral += hata * dt
            integral = max(-MAKS_INTEGRAL, min(MAKS_INTEGRAL, integral))

            duzeltme = DUZELTME_KAZANCI * hata + DUZELTME_KAZANCI_I * integral
            duzeltme = max(-MAKS_DUZELTME, min(MAKS_DUZELTME, duzeltme))
        else:
            duzeltme = 0.0

        sol_duty = max(0, min(100, temel_sol - duzeltme))
        sag_duty = max(0, min(100, temel_sag + duzeltme))
        pwm_a.ChangeDutyCycle(sol_duty)
        pwm_b.ChangeDutyCycle(sag_duty)

        time.sleep(0.03)

    motorlari_durdur(pwm_a, pwm_b)
    print("Zaman bazli duz gitme tamamlandi.")


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

    # HIZLI MOD: renk sensoru okumasini kapat - heading/mesafe verisinin
    # cok daha guncel gelmesini saglar.
    bridge.request_fast_mode()
    time.sleep(0.1)

    pwm_a, pwm_b = motorlari_ayarla()

    try:
        # ---- 1) 40 derece saga don ----
        print("\n=== 1. DONUS: 40 derece saga ===")
        if not guvenli_donus(38, "sag", bridge, pwm_a, pwm_b):
            print("Test durduruldu (1. donus basarisiz).")
            return

        # ---- 2) 0.5 saniye duz git (zaman bazli) ----
        print("\n=== 2. ADIM: 0.5 saniye duz git ===")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.5)

        # ---- 3) 50 derece saga don ----
        print("\n=== 3. DONUS: 50 derece saga ===")
        if not guvenli_donus(48, "sag", bridge, pwm_a, pwm_b):
            print("Test durduruldu (3. donus basarisiz).")
            return

        # ---- 4) Hedefe 65 cm kalana kadar duz git ----
        print("\n=== 4. ADIM: Hedefe 65cm kalana kadar git ===")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=60.0)
        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        # ---- 5) 75 derece sola don ----
        print("\n=== 5. DONUS: 75 derece sola ===")
        if not guvenli_donus(74, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (5. donus basarisiz).")
            return

        # ---- 6) 75 derece saga don ----
        print("\n=== 6. DONUS: 75 derece saga ===")
        if not guvenli_donus(72, "sag", bridge, pwm_a, pwm_b):
            print("Test durduruldu (6. donus basarisiz).")
            return

        # ---- 7) Hedefe 40 cm kalana kadar duz git ----
        print("\n=== 7. ADIM: Hedefe 40cm kalana kadar git ===")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=40.0)
        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        # ---- 8) 85 derece sola don ----
        print("\n=== 8. DONUS: 85 derece sola ===")
        if not guvenli_donus(85, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (8. donus basarisiz).")
            return

        # ---- 9) 85 derece saga don ----
        print("\n=== 9. DONUS: 85 derece saga ===")
        if not guvenli_donus(82, "sag", bridge, pwm_a, pwm_b):
            print("Test durduruldu (9. donus basarisiz).")
            return

        # ---- 10) Hedefe 20 cm kalana kadar duz git ----
        print("\n=== 10. ADIM: Hedefe 20cm kalana kadar git ===")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=20.0)
        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        # ---- 11) 100 derece sola don ----
        print("\n=== 11. DONUS: 100 derece sola ===")
        if not guvenli_donus(97, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (11. donus basarisiz).")
            return

        # ---- 12) 10 derece saga don ----
        print("\n=== 12. DONUS: 10 derece saga ===")
        if not guvenli_donus(9, "sag", bridge, pwm_a, pwm_b):
            print("Test durduruldu (12. donus basarisiz).")
            return

        # ---- 13) 2 saniye duz git (zaman bazli) ----
        print("\n=== 13. ADIM: 2 saniye duz git ===")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 2.0)

        # ---- 14) 25 derece sola don ----
        print("\n=== 14. DONUS: 25 derece sola ===")
        if not guvenli_donus(25, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (14. donus basarisiz).")
            return

        print("\nOzel rota testi tamamlandi.")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
