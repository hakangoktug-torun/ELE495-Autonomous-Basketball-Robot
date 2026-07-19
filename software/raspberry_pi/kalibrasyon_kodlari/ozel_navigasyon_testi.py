"""
ELE495 - Ozel Navigasyon Test Rotasi (2 kullanici girdili)
test_surus.py'nin kare rotasindan FARKLI, ozel bir hareket dizisi dener.

Bu dosya IKI SEKILDE kullanilabilir (firlaticisiz_atis_simulasyon_test.py ile
ayni mantik):
  1) Komut satirindan direkt calistirilabilir (input() ile sorar):
         python3 ozel_navigasyon_testi.py
  2) Baska bir kod (Flask GUI) tarafindan import edilip
     calistir_ozel_rota() cagrilabilir - input() yerine caller'in verdigi
     callback fonksiyonlari (aci_getir_fn, olay_fn) kullanilir.

ADIMLAR:
  1) Saga 30 derece don
  2) 0.5 saniye duz git
  3) Saga 60 derece don
  4) 0.3 saniye duz git
  5) Kullanicidan yon/aci sor (1. serbest donus)
  6) O aciyla don
  7) Ayni aciyla TERS yone donup eski haline geri gel
  8) Ultrasonik mesafe oku - onde 10cm'den fazla acikliksa, 10cm ileri git
  9) Kullanicidan yon/aci sor (2. serbest donus)
  10) O aciyla don
  11) Bitir

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve test_surus.py ile ayni yerde olmali)
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import (
    motorlari_ayarla, motorlari_durdur, aci_farki, SERIAL_PORT
)
from test_surus import (
    guvenli_donus, ileri_yon_ayarla, ileri_git_sabit_mesafe,
    SOL_HIZ, SAG_HIZ, DUZELTME_KAZANCI, DUZELTME_KAZANCI_I, MAKS_DUZELTME,
    MAKS_INTEGRAL,
)

MIN_ACIKLIK_CM = 10.0     # onde bu kadardan fazla aciklik varsa ilerle
ILERLEME_MESAFESI_CM = 10.0  # ne kadar ilerlenecek


def ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure_saniye):
    """
    Belirtilen sure boyunca duz ileri gider (mesafe sensoru KULLANILMAZ,
    sadece zaman). BNO055 heading feedback ile saga/sola kaymayi anlik
    olarak duzeltir.
    """
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


def calistir_ozel_rota(bridge, pwm_a, pwm_b, aci_getir_fn, olay_fn=print):
    """
    Yukaridaki 10 adimlik hareket dizisini calistirir - CLI ve GUI arasinda
    PAYLASILAN tek mantik burasi.

    aci_getir_fn(baglam) -> (yon, aci)
        baglam: {"asama": 1 veya 2, "mesaj": "..."} - hangi asamada
        oldugumuzu ve kullaniciya gosterilecek aciklamayi tasir.
        Cagrildiginda BLOKE OLABILIR (CLI'da input() bekler, GUI'de web'den
        cevap gelene kadar bekler).
    olay_fn(mesaj)
        Her onemli adimda cagrilir (log/tarihce icin). Varsayilan: print.

    Donus deger: True (tum adimlar basarili) / False (bir adimda basarisiz oldu)
    """
    # ---- 1) Saga 30 derece don ----
    olay_fn("1. adim: 30 derece saga donuluyor...")
    if not guvenli_donus(30, "sag", bridge, pwm_a, pwm_b):
        olay_fn("1. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 2) 0.5 saniye duz git ----
    olay_fn("2. adim: 0.5 saniye duz gidiliyor...")
    ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.5)

    # ---- 3) Saga 60 derece don ----
    olay_fn("3. adim: 60 derece saga donuluyor...")
    if not guvenli_donus(60, "sag", bridge, pwm_a, pwm_b):
        olay_fn("3. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 4) 0.3 saniye duz git ----
    olay_fn("4. adim: 0.3 saniye duz gidiliyor...")
    ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.3)

    # ---- 5) Kullanicidan yon/aci sor (1. serbest donus) ----
    olay_fn("5. adim: ilk serbest donus icin yon/aci bekleniyor...")
    yon1, aci1 = aci_getir_fn({
        "asama": 1,
        "mesaj": "1. serbest donus - istedigin yon ve aciyi gir",
    })

    # ---- 6) O aciyla don ----
    olay_fn(f"6. adim: {aci1} derece {yon1} yone donuluyor...")
    if not guvenli_donus(aci1, yon1, bridge, pwm_a, pwm_b):
        olay_fn("6. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 7) Ayni aciyla TERS yone donup eski haline geri gel ----
    ters_yon1 = "sol" if yon1 == "sag" else "sag"
    olay_fn(f"7. adim: {aci1} derece {ters_yon1} yone donup eski haline geri donuluyor...")
    if not guvenli_donus(aci1, ters_yon1, bridge, pwm_a, pwm_b):
        olay_fn("7. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 8) Ultrasonik mesafe oku - onde 10cm'den fazla aciklik varsa 10cm ilerle ----
    mesafe = bridge.get_distance()
    olay_fn(f"8. adim: ultrasonik mesafe olculdu: {mesafe} cm")
    if mesafe is not None and mesafe > MIN_ACIKLIK_CM:
        olay_fn(f"8. adim: onde yeterli aciklik var, {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
        ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)
    else:
        olay_fn("8. adim: onde yeterli aciklik yok (10cm ve altinda), ilerleme atlaniyor.")

    # ---- 9) Kullanicidan yon/aci sor (2. serbest donus) ----
    olay_fn("9. adim: ikinci serbest donus icin yon/aci bekleniyor...")
    yon2, aci2 = aci_getir_fn({
        "asama": 2,
        "mesaj": "2. serbest donus - istedigin yon ve aciyi gir",
    })

    # ---- 10) O aciyla don ----
    olay_fn(f"10. adim: {aci2} derece {yon2} yone donuluyor...")
    if not guvenli_donus(aci2, yon2, bridge, pwm_a, pwm_b):
        olay_fn("10. adim basarisiz oldu, durduruluyor.")
        return False

    olay_fn("Ozel navigasyon test rotasi tamamlandi.")
    return True


# ---------------------------------------------------------------------------
# CLI modu
# ---------------------------------------------------------------------------

def cli_aci_getir(baglam):
    print(f"\n=== {baglam['mesaj']} ===")
    while True:
        yon = input("Donus yonu (sol/sag): ").strip().lower()
        if yon in ("sol", "sag"):
            break
        print("Gecersiz - 'sol' ya da 'sag' yaz.")

    while True:
        aci_str = input("Donus acisi (derece, orn: 30): ").strip()
        try:
            aci = float(aci_str)
            if aci > 0:
                break
            print("Aci pozitif bir sayi olmali.")
        except ValueError:
            print("Gecerli bir sayi gir.")

    return yon, aci


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

    try:
        calistir_ozel_rota(bridge, pwm_a, pwm_b, aci_getir_fn=cli_aci_getir, olay_fn=print)

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
