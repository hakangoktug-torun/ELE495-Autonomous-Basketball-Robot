"""
Kapali dongu (closed-loop) donus kontrolu - BNO055 feedback ile
Sabit sure yerine, hedef aciya ulasana kadar doner ve otomatik durur.

Kullanim ornegi (dosyanin sonundaki main() icinde):
    donus_yap(90, yon="sol")   # 90 derece sola don

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
heading_bridge.py'nin bir ust dizinde (software/raspberry_pi/) oldugunu varsayiyorum.
"""

import sys
import os
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from heading_bridge import HeadingBridge

# ---------- Motor pinleri ----------
IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16

# ---------- Hiz ayarlari ----------
HIZ_NORMAL = 30      # ana donus hizi
HIZ_YAVAS = 15        # hedefe yaklasirken yavaslama hizi
YAVASLAMA_ESIGI = 30.0  # hedefe kalan derece bu esigin altina dusunce yavasla
TOLERANS = 2.0         # hedefe bu kadar derece yakinsa "ulasti" say
ZAMAN_ASIMI = 8.0      # saniye - sensor/motor sorununda sonsuz donmeyi engeller

SERIAL_PORT = "/dev/ttyUSB0"

# ---------- Ince duzeltme (fine correction) ayarlari ----------
FINE_TOLERANS = 1.0        # bu derecenin altindaki hata artik kabul edilir
DUZELTME_HIZ = 30           # duzeltme atislari icin dusuk hiz (motor nonlinearity nedeniyle cok dusuk olmasin)
DUZELTME_MIN_SURE = 0.03    # saniye - en kisa duzeltme atisi
DUZELTME_MAX_SURE = 0.15    # saniye - en uzun duzeltme atisi
DUZELTME_SETTLE = 0.4       # her atistan sonra olcum oncesi bekleme (magnetometer/motor sakinlessin)
MAKS_DUZELTME_DENEME = 12   # sonsuz salinim olmasin diye deneme sinirI


def kalibrasyon_bekle(bridge, kontrol_araligi=2.0):
    """
    Rotasyon testi baslamadan once BNO055 kalibrasyon durumunu tekrar tekrar
    gosterir ve kullaniciya sorar. Kullanici 'yes' yazana kadar bekler.
    Sensoru '8' cizer gibi cevirmen gerekir - magnetometer boyle kalibre olur.
    """
    print("\n=== KALIBRASYON KONTROLU ===")
    print("Sensoru (ya da uzerine monteli oldugu robotu) elinizle havada")
    print("'8' cizer gibi yavasca, farkli yonlerde cevirin.\n")

    while True:
        bridge.request_calibration_status()
        time.sleep(0.3)  # cevabin _read_loop icinde islenmesini bekle

        cal = bridge.get_calibration()
        print(f"Guncel kalibrasyon -> sys={cal['sys']} gyro={cal['gyro']} "
              f"accel={cal['accel']} mag={cal['mag']}  (0=kotu, 3=mukemmel)")

        cevap = input("Donus testine devam edilsin mi? (devam icin 'yes' yaz, "
                       "beklemek icin Enter'a bas): ").strip().lower()

        if cevap == "yes":
            print("Devam ediliyor.\n")
            return True

        time.sleep(max(0.0, kontrol_araligi))


def aci_farki(baslangic, bitis):
    """0-360 derece sarmalini dogru hesaplayan aci farki (-180, +180] araliginda."""
    fark = bitis - baslangic
    if fark < -180:
        fark += 360
    elif fark > 180:
        fark -= 360
    return fark


def motorlari_ayarla():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
        GPIO.setup(p, GPIO.OUT)
    pwm_a = GPIO.PWM(ENA, 1000)
    pwm_b = GPIO.PWM(ENB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b


def motorlari_durdur(pwm_a, pwm_b):
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.output(p, GPIO.LOW)


def donus_yonu_ayarla(yon):
    """yon='sol' -> saat yonu tersine (heading azalir, onceki testlerdeki gibi)
       yon='sag' -> saat yonune (heading artar)"""
    if yon == "sol":
        GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
    elif yon == "sag":
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    else:
        raise ValueError("yon 'sol' ya da 'sag' olmali")


def ince_duzeltme_yap(bridge, pwm_a, pwm_b, hedef_isaretli, toplam_donus, onceki_heading):
    """
    Ana donus bittikten sonra, kalan hata FINE_TOLERANS'in ustundeyse
    kisa duzeltme atislari (pulse) yaparak hatayi azaltmaya calisir.
    Her atistan sonra durup olcum alir, gerekirse ters yonde tekrar dener.

    Donus deger: (guncellenmis toplam_donus, guncellenmis onceki_heading)
    """
    for deneme in range(1, MAKS_DUZELTME_DENEME + 1):
        hata_isaretli = hedef_isaretli - toplam_donus

        if abs(hata_isaretli) <= FINE_TOLERANS:
            print(f"Ince duzeltme tamamlandi ({deneme - 1} atis sonrasi). "
                  f"Kalan hata: {abs(hata_isaretli):.2f} derece")
            return toplam_donus, onceki_heading

        yon_bu_atis = "sag" if hata_isaretli > 0 else "sol"
        pulse_sure = min(DUZELTME_MAX_SURE, max(DUZELTME_MIN_SURE, abs(hata_isaretli) / 200.0))

        print(f"  Duzeltme atisi #{deneme}: hata={hata_isaretli:.2f} derece, "
              f"yon={yon_bu_atis}, sure={pulse_sure:.3f}s")

        donus_yonu_ayarla(yon_bu_atis)
        pwm_a.ChangeDutyCycle(DUZELTME_HIZ)
        pwm_b.ChangeDutyCycle(DUZELTME_HIZ)
        time.sleep(pulse_sure)
        motorlari_durdur(pwm_a, pwm_b)

        time.sleep(DUZELTME_SETTLE)

        yeni_heading = bridge.get_heading()
        if yeni_heading is not None and onceki_heading is not None:
            toplam_donus += aci_farki(onceki_heading, yeni_heading)
            onceki_heading = yeni_heading

    hata_isaretli = hedef_isaretli - toplam_donus
    print(f"UYARI: Maksimum duzeltme denemesi ({MAKS_DUZELTME_DENEME}) doldu. "
          f"Kalan hata: {abs(hata_isaretli):.2f} derece (hedeflenen: {FINE_TOLERANS} derece)")
    return toplam_donus, onceki_heading


def donus_yap(hedef_derece, yon="sol", bridge=None):
    """
    hedef_derece: kac derece donulecek (pozitif sayi, yon parametresi ile yon belirlenir)
    yon: 'sol' ya da 'sag'
    bridge: disaridan HeadingBridge nesnesi verilebilir (main() disinda cagrilirsa)

    Donus deger: gercekte donulen toplam derece (float)
    """
    kendi_bridge = False
    if bridge is None:
        bridge = HeadingBridge(port=SERIAL_PORT)
        bridge.start()
        kendi_bridge = True

        print("BNO055 baglantisi bekleniyor...")
        for _ in range(50):
            if not bridge.is_stale(max_age_sec=1.0):
                break
            time.sleep(0.1)

        if bridge.is_stale(max_age_sec=1.0):
            print("UYARI: BNO055'ten veri gelmiyor, baglantiyi kontrol et.")
            if kendi_bridge:
                bridge.stop()
            return 0.0

    pwm_a, pwm_b = motorlari_ayarla()

    try:
        kalibrasyon_bekle(bridge)

        toplam_donus = 0.0
        onceki_heading = bridge.get_heading()
        if onceki_heading is None:
            print("HATA: Heading okunamadi, donus iptal edildi.")
            return 0.0

        donus_yonu_ayarla(yon)
        pwm_a.ChangeDutyCycle(HIZ_NORMAL)
        pwm_b.ChangeDutyCycle(HIZ_NORMAL)

        yavas_moda_gecildi = False
        baslangic_zamani = time.time()

        while True:
            if time.time() - baslangic_zamani > ZAMAN_ASIMI:
                print("UYARI: Zaman asimi, donus zorla durduruldu (sensor/motor sorunu olabilir).")
                break

            simdiki_heading = bridge.get_heading()
            if simdiki_heading is not None:
                adim_farki = aci_farki(onceki_heading, simdiki_heading)
                toplam_donus += adim_farki
                onceki_heading = simdiki_heading

            kalan = hedef_derece - abs(toplam_donus)

            if kalan <= TOLERANS:
                break

            # Hedefe yaklasinca yavasla (overshoot'u azaltmak icin)
            if kalan <= YAVASLAMA_ESIGI and not yavas_moda_gecildi:
                pwm_a.ChangeDutyCycle(HIZ_YAVAS)
                pwm_b.ChangeDutyCycle(HIZ_YAVAS)
                yavas_moda_gecildi = True

            time.sleep(0.02)  # ~50Hz kontrol dongusu

        motorlari_durdur(pwm_a, pwm_b)

        # Motor calisirken olusan manyetik girisimin sonmesi icin daha uzun bekle.
        # 0.3s yetersizdi - motor durduktan sonra da manyetik alan bir sure etkili kalabiliyor.
        SETTLE_SURESI = 1.5
        print(f"Motor durdu, magnetometer'in sakinlesmesi icin {SETTLE_SURESI}s bekleniyor...")
        time.sleep(SETTLE_SURESI)

        # Son okumayi da kumulatif toplama ekle
        son_heading = bridge.get_heading()
        if son_heading is not None and onceki_heading is not None:
            toplam_donus += aci_farki(onceki_heading, son_heading)

        # Kalibrasyon durumunu kontrol et - dusukse manyetik girisim teorisini dogrular
        bridge.request_calibration_status()
        time.sleep(0.2)  # cevabin gelmesini bekle (_read_loop icinde yazdirilir)

        print(f"Ana donus sonucu: {toplam_donus:.1f} derece "
              f"(fark: {abs(hedef_derece - abs(toplam_donus)):.1f} derece)")

        # ---- Ince duzeltme: hata FINE_TOLERANS'in altina inene kadar kucuk atislarla duzelt ----
        hedef_isaretli = -hedef_derece if yon == "sol" else hedef_derece
        toplam_donus, onceki_heading = ince_duzeltme_yap(
            bridge, pwm_a, pwm_b, hedef_isaretli, toplam_donus, onceki_heading
        )

        print(f"Hedef: {hedef_derece} derece, gercekte donulen: {toplam_donus:.1f} derece "
              f"(fark: {abs(hedef_derece - abs(toplam_donus)):.1f} derece)")

        return toplam_donus

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        if kendi_bridge:
            bridge.stop()


def main():
    # Test: 360 derece (tam tur) sola don
    # Robotun uzerine bir ok/isaret koyup, donus oncesi ve sonrasi
    # ayni yone bakip bakmadigini gozle kontrol edebilirsin.
    donus_yap(360, yon="sol")


if __name__ == "__main__":
    main()
