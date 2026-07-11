"""
ELE495 - Interaktif Rota Testi
test_surus.py'deki 'kare cizme' mantiginin ayni, ama donus acisi/yonu SABIT
(20 derece) DEGIL - her hedefe ulastiktan sonra SENDEN sorulur.

Senaryo:
  1) Bir cisim 20 cm'ye kadar yaklasana kadar duz git (1. hedef)
  2) Sana hangi yone, kac derece test etmek istedigini sorar, o kadar doner,
     SONRA AYNI ACIYI TERS YONDE UYGULAYIP BASLANGIC YONUNE GERI DONER
  3) Bir cisim 20 cm'ye kadar yaklasana kadar duz git (2. hedef, ayni yonde)
  4) Tekrar sorar, test eder, geri doner
  5) 3. hedef
  6) Tekrar sorar, test eder, geri doner
  7) 4. hedef
  8) Biter (4. hedeften sonra test sormuyoruz, is bitti)

Bu, DEMO icin degil - senin robotu test etmen icin.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve test_surus.py ile ayni yerde olmali)

GUNCELLEME NOTU: Otomatik isinma hareketi (isinma_yap) KALDIRILDI -
test_surus.py'deki ayni degisiklikle tutarli olsun diye. EEPROM'dan
kalibrasyon zaten yukleniyor ve hizli mod (renk sensoru kapali) veri
kalitesini yeterince artirdigi icin, isinma hareketinin sagladigi kucuk
fayda, robotun baslangic yonunde istenmeyen kaymaya yol acma riskine
degmiyordu.
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import (
    donus_yap, motorlari_ayarla, motorlari_durdur, aci_farki,
    IN1, IN2, IN3, IN4, ENA, ENB, SERIAL_PORT
)
from test_surus import (
    ileri_git_engel_bulunca, ileri_yon_ayarla,
)

TOPLAM_HEDEF_SAYISI = 4
ENGEL_ESIGI_CM = 20.0  # bu scriptte ozel olarak 20cm - test_surus.py'nin varsayilan 30cm'sinden farkli


def test_aci_ve_geri_don(bridge, pwm_a, pwm_b, hedef_no):
    """
    Kullanicidan test edilecek yon ve aciyi sorar, o kadar doner, sonra
    AYNI aciyi TERS yonde uygulayarak baslangic yonune geri doner.

    Bu, test_surus.py'deki 'kucuk_aci_test_ve_ana_donus' ile ayni mantik,
    ama sabit 20 derece yerine SENDEN alinan yon/aci kullanilir, ve
    donuldukten sonra yeni bir 'ana' yone gecmek yerine BASLANGIC yonune
    geri donulur (robot bir sonraki hedefi ayni yonde aramaya devam eder).

    Donus deger: True (basarili) / False (basarisiz)
    """
    yon, aci = yon_ve_aci_sor(hedef_no)

    ters_yon = "sol" if yon == "sag" else "sag"

    print(f"\n{aci} derece {yon} yone donuluyor (test)...")
    if not guvenli_donus_interaktif(aci, yon, bridge, pwm_a, pwm_b):
        print(f"Test donusu ({aci} derece {yon}) basarisiz oldu.")
        return False

    print(f"Simdi {aci} derece {ters_yon} yone donup baslangic yonune donuluyor...")
    if not guvenli_donus_interaktif(aci, ters_yon, bridge, pwm_a, pwm_b):
        print(f"Geri donus ({aci} derece {ters_yon}) basarisiz oldu.")
        return False

    return True


def yon_ve_aci_sor(hedef_no):
    """Kullanicidan donus yonu ve acisini ister, gecerli girdi alana kadar tekrar sorar."""
    print(f"\n=== {hedef_no}. HEDEFE ULASILDI ===")
    while True:
        yon = input("Bu konumda test edilecek donus yonu (sol/sag): ").strip().lower()
        if yon in ("sol", "sag"):
            break
        print("Gecersiz - 'sol' ya da 'sag' yaz.")

    while True:
        aci_str = input("Test edilecek aci (derece, orn: 45): ").strip()
        try:
            aci = float(aci_str)
            if aci > 0:
                break
            print("Aci pozitif bir sayi olmali.")
        except ValueError:
            print("Gecerli bir sayi gir.")

    return yon, aci


def guvenli_donus_interaktif(aci, yon, bridge, pwm_a, pwm_b, maks_deneme=2):
    """test_surus.py'deki guvenli_donus ile ayni mantik - basarisizsa tekrar dener.
    otonom=True: EEPROM'dan kalibrasyon yukluyse, her seferinde 'yes' yazmani
    beklemez, otomatik olarak devam eder."""
    for deneme in range(1, maks_deneme + 1):
        sonuc = donus_yap(aci, yon=yon, bridge=bridge, pwm_a=pwm_a, pwm_b=pwm_b, otonom=True)

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

    # HIZLI MOD: renk sensoru okumasini kapat - heading/mesafe verisinin
    # cok daha guncel gelmesini saglar (test_surus.py ile tutarli).
    bridge.request_fast_mode()
    time.sleep(0.1)

    pwm_a, pwm_b = motorlari_ayarla()

    # NOT: Otomatik isinma hareketi (kucuk sallanma) KALDIRILDI - bkz. dosya
    # basindaki guncelleme notu. Gerekirse yeniden aktiflestirmek icin:
    #   from donus_kapali_dongu import isinma_yap
    #   isinma_yap(bridge, pwm_a, pwm_b)

    try:
        for hedef_no in range(1, TOPLAM_HEDEF_SAYISI + 1):
            print(f"\n--- {hedef_no}. hedef araniyor ---")
            bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=ENGEL_ESIGI_CM)

            if not bulundu:
                print(f"{hedef_no}. hedef bulunamadi, test durduruldu.")
                return

            if hedef_no == TOPLAM_HEDEF_SAYISI:
                # Son hedefe ulasildi, artik test sormuyoruz - is bitti.
                break

            if not test_aci_ve_geri_don(bridge, pwm_a, pwm_b, hedef_no):
                print("Test donusu basarisiz oldugu icin test durduruldu.")
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
