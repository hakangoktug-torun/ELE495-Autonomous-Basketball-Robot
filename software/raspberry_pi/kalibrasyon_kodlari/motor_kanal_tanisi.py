"""
ELE495 - Motor Kanali Tanisi (Sol/Sag izole test)
Amac: hangi TARAFIN (sol/sag) ne siklikta arizalandigini KESIN olarak
tespit etmek. Rastgele karisik donuslerden degil, TEK bir yonu tekrar
tekrar calistirip GOZLE izleyerek veri topluyoruz.

NASIL CALISIR:
  - Once SOL yonde N kez, kisa sureli (1 saniyelik) tam hizda calisir.
    Her denemede senden "tum tekerlekler dondu mu?" (e/h) diye sorar.
  - Sonra ayni sekilde SAG yonde test eder.
  - Sonunda her yon icin basari oranini (X/N) gosterir.

Bunu yaparken robotu YERDEN KALDIR (tekerlekler havada, hicbir yuk/surtunme
olmadan) - boylece "motor gucu yetersiz" ihtimalini tamamen eliyoruz, geriye
sadece "baglanti/motor arizasi" ihtimali kaliyor.

Kullanim:
    python3 motor_kanal_tanisi.py
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from donus_kapali_dongu import (
    motorlari_ayarla, motorlari_durdur, donus_yonu_ayarla,
    IN1, IN2, IN3, IN4
)

HIZ = 36           # HIZ_NORMAL ile ayni (guncellendi - 1.2x)
CALISMA_SURESI = 1.0  # saniye - her denemede motor bu kadar sure calisir
DENEME_SAYISI = 15    # her yon icin kac kez tekrar edilecek


def tek_yon_test_et(pwm_a, pwm_b, taraf_adi, yon):
    """
    'sol' yon -> IN1=LOW,IN2=HIGH ve IN3=LOW,IN4=HIGH
    'sag' yon -> IN1=HIGH,IN2=LOW ve IN3=HIGH,IN4=LOW
    Bu ikisi FARKLI elektriksel kombinasyonlar oldugu icin, sol/sag
    testleri motorlari FARKLI durumlarda calistirmis olur - bu da sorunun
    belirli bir sinyal kombinasyonuna mi (orn. hep IN2=HIGH durumunda mi)
    bagli oldugunu ortaya cikarabilir.
    """
    basarili = 0
    basarisiz = 0

    print(f"\n=== {taraf_adi} icin test basliyor ({DENEME_SAYISI} deneme) ===")
    print("Robotu yerden kaldirdigindan emin ol (tekerlekler havada, yuksuz).\n")
    input("Hazir oldugunda Enter'a bas...")

    for i in range(1, DENEME_SAYISI + 1):
        donus_yonu_ayarla(yon)
        pwm_a.ChangeDutyCycle(HIZ)
        pwm_b.ChangeDutyCycle(HIZ)
        time.sleep(CALISMA_SURESI)
        motorlari_durdur(pwm_a, pwm_b)
        time.sleep(0.3)

        while True:
            cevap = input(f"  Deneme {i}/{DENEME_SAYISI}: TUM tekerlekler duzgun dondu mu? "
                          f"(e=evet, h=hayir): ").strip().lower()
            if cevap in ("e", "h"):
                break
            print("  Gecerli bir girdi degil - 'e' ya da 'h' yaz.")

        if cevap == "e":
            basarili += 1
        else:
            basarisiz += 1
            not_str = input("    (Istersen hangi tekerin/nasil basarisiz oldugunu yaz, "
                             "bos gecebilirsin): ").strip()
            if not_str:
                print(f"    -> Not edildi: {not_str}")

    print(f"\n{taraf_adi} SONUC: {basarili}/{DENEME_SAYISI} basarili "
          f"({100*basarili/DENEME_SAYISI:.0f}%)")
    return basarili, basarisiz


def main():
    pwm_a, pwm_b = motorlari_ayarla()

    try:
        print("=== MOTOR KANALI TANISI ===")
        print("Bu test, sol ve sag yonleri ayri ayri tekrar tekrar calistirip")
        print("hangi yonun ne siklikta arizalandigini olcer.\n")

        sol_basarili, sol_basarisiz = tek_yon_test_et(pwm_a, pwm_b, "SOL YON", "sol")
        sag_basarili, sag_basarisiz = tek_yon_test_et(pwm_a, pwm_b, "SAG YON", "sag")

        print("\n\n=== GENEL SONUC ===")
        print(f"SOL yon: {sol_basarili}/{DENEME_SAYISI} basarili")
        print(f"SAG yon: {sag_basarili}/{DENEME_SAYISI} basarili")

        if sol_basarisiz > sag_basarisiz * 1.5:
            print("\nSOL yon ciddi sekilde daha fazla basarisiz oldu - bu yonde "
                  "aktif olan motor/kablo grubunu incele.")
        elif sag_basarisiz > sol_basarisiz * 1.5:
            print("\nSAG yon ciddi sekilde daha fazla basarisiz oldu - bu yonde "
                  "aktif olan motor/kablo grubunu incele.")
        else:
            print("\nIki yon de benzer basarisizlik oraninda - sorun belirli bir "
                  "yon/kanala bagli degil, daha genel bir baglanti/guc sorunu olabilir "
                  "(orn. tum sistemi besleyen ana baglanti noktalari).")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        print("\nGPIO temizlendi.")


if __name__ == "__main__":
    main()
