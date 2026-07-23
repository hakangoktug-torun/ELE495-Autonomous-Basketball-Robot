"""
ELE495 - Hareket Guvenilirlik Testi (donus + duz gitme)
Amac: aracin GERCEK kapali dongu hareket fonksiyonlarini (guvenli_donus,
duz gitme) tekrar tekrar calistirip, stall/tutukluk olup olmadigini SENIN
GOZLEMLEMENLE dogrulamak. motor_kanal_tanisi.py'den farki: bu, ham GPIO
darbeleri degil, GERCEKTEN kullanilan (sensor feedback'li) fonksiyonlari
test eder - yani "gercek kosullarda" davranisi gorursun.

Her turda:
  1) 90 derece SAGA don
  2) 90 derece SOLA don (baslangica geri gel)
  3) ~1 saniye duz ileri git

Her adimdan sonra "duzgun oldu mu? (e/h)" diye sorar, sonunda ozet basar.
Kod, kendi icindeki stall/hiz anomalisi tespitini de otomatik calistirir
(donus_yap ve ileri_git_engel_bulunca'nin zaten yaptigi gibi) - yani
"(STALL supheli...)" ya da "(HIZ ANOMALISI...)" gibi satirlar gorursen,
kod BUNU zaten kendisi tespit etmis demektir; senin verdigin e/h cevaplari
buna ek, bagimsiz bir dogrulama.

Kullanim:
    python3 hareket_guvenilirlik_testi.py
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, aci_farki, SERIAL_PORT
from test_surus import (
    guvenli_donus, ileri_yon_ayarla,
    SOL_HIZ, SAG_HIZ, DUZELTME_KAZANCI, DUZELTME_KAZANCI_I, MAKS_DUZELTME,
    MAKS_INTEGRAL,
)

DENEME_SAYISI = 10
ILERI_SURESI = 1.0  # saniye - her turda ne kadar duz gidilecek


def ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure_saniye):
    """Belirtilen sure boyunca duz ileri gider, BNO055 ile saga/sola kaymayi duzeltir."""
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

        pwm_a.ChangeDutyCycle(max(0, min(100, temel_sol - duzeltme)))
        pwm_b.ChangeDutyCycle(max(0, min(100, temel_sag + duzeltme)))
        time.sleep(0.03)

    motorlari_durdur(pwm_a, pwm_b)


def sor(mesaj):
    while True:
        cevap = input(f"{mesaj} (e/h): ").strip().lower()
        if cevap in ("e", "h"):
            return cevap == "e"
        print("Gecerli bir cevap degil - 'e' ya da 'h' yaz.")


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

    bridge.request_fast_mode()
    time.sleep(0.1)

    pwm_a, pwm_b = motorlari_ayarla()

    sonuclar = {"saga_donus": [0, 0], "sola_donus": [0, 0], "duz_gitme": [0, 0]}  # [basarili, toplam]

    print(f"\n=== HAREKET GUVENILIRLIK TESTI ({DENEME_SAYISI} tur) ===")
    print("Her adimdan sonra robotu gozle izleyip degerlendirecek, cevabini gireceksin.\n")
    input("Hazir oldugunda Enter'a bas...")

    try:
        for tur in range(1, DENEME_SAYISI + 1):
            print(f"\n--- Tur {tur}/{DENEME_SAYISI} ---")

            print("90 derece saga donuluyor...")
            guvenli_donus(90, "sag", bridge, pwm_a, pwm_b)
            basarili = sor("  Saga donus TUM tekerleklerle duzgun oldu mu, stall/tutukluk yok muydu?")
            sonuclar["saga_donus"][0] += int(basarili)
            sonuclar["saga_donus"][1] += 1

            print("90 derece sola donuluyor (baslangica donus)...")
            guvenli_donus(90, "sol", bridge, pwm_a, pwm_b)
            basarili = sor("  Sola donus TUM tekerleklerle duzgun oldu mu, stall/tutukluk yok muydu?")
            sonuclar["sola_donus"][0] += int(basarili)
            sonuclar["sola_donus"][1] += 1

            print(f"{ILERI_SURESI}s duz ileri gidiliyor...")
            ileri_git_sabit_sure(bridge, pwm_a, pwm_b, ILERI_SURESI)
            basarili = sor("  Duz gitme TUM tekerleklerle duzgun oldu mu, stall/tutukluk yok muydu?")
            sonuclar["duz_gitme"][0] += int(basarili)
            sonuclar["duz_gitme"][1] += 1

    except KeyboardInterrupt:
        print("\n\nErken durduruldu.")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()

    print("\n\n=== SONUC ===")
    for kategori, (basarili, toplam) in sonuclar.items():
        oran = (100 * basarili / toplam) if toplam else 0
        print(f"{kategori}: {basarili}/{toplam} basarili (%{oran:.0f})")


if __name__ == "__main__":
    main()
