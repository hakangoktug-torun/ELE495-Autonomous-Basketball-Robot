"""
ELE495 - Test Surusu
Senaryo:
  1) Bir cisim 30 cm'ye kadar yaklasana kadar duz git
  2) 90 derece sola don (kapali dongu, BNO055 ile)
  3) 20 cm ileri git
  4) Tekrar 90 derece sola don
  5) 20 cm ileri git
  6) Dur

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py ve robot_bridge.py ile ayni yerde olmali)

ONEMLI VARSAYIM: Duz ileri gitme icin motor yon pinlerini tahmin ederek yazdim
(IN1=HIGH,IN2=LOW ve IN3=HIGH,IN4=LOW seklinde - donus kodundaki 'her iki motor
ayni sinyali alir' mantigina ters, cunku duz gitmede motorlar TERS yonde
komutlanmali - fiziksel montaj karsilikli oldugu icin). Eger robot ilk testte
GERI giderse, ILERI_IN_PATTERN degiskenini asagida ters cevir.
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

# ---------- Duz gitme kapali dongu duzeltme ayarlari ----------
DUZELTME_KAZANCI = 0.4   # derece basina duty duzeltme miktari - once kucuk basla, gerekirse artir
MAKS_DUZELTME = 10.0     # duty cinsinden - tekerlek hizinin ne kadar degisebilecegi ustsiniri

# ---------- Duz gitme hiz/kalibrasyon ayarlari ----------
# Onceki kalibrasyon notlarindan: SOL_HIZ=25, SAG_HIZ=28 (+3 offset sag kaymayi
# duzeltiyordu), CM_PER_SANIYE=23.5 (tam bu degerler duz gitme icin gecerliydi).
SOL_HIZ = 25
SAG_HIZ = 28
CM_PER_SANIYE = 23.5

ENGEL_ESIGI_CM = 30.0     # bu mesafenin altina inince dur
YAVASLAMA_MESAFE_FARKI = 15.0  # esige bu kadar cm kala yavasla (1. kademe)
HIZ_YAVAS_ENGEL = 18       # 1. kademe yavaslama hizi
COK_YAVAS_MESAFE_FARKI = 5.0   # esige bu kadar cm kala IYICE yavasla (2. kademe)
HIZ_COK_YAVAS_ENGEL = 13   # 2. kademe (son yaklasim) hizi - overshoot'u minimize eder
ILERI_ADIM_CM = 20.0      # her iki donusten sonra gidilecek mesafe
MAKS_ENGEL_BEKLEME = 20.0  # saniye - engel hic bulunamazsa guvenlik siniri


def ileri_yon_ayarla():
    """
    VARSAYIM: Duz ileri gitme icin motor pinleri. Donus kodunda (donus_yonu_ayarla)
    'sol' icin IN1=LOW,IN2=HIGH ve IN3=LOW,IN4=HIGH kullanilmisti (ikisi ayni yonde
    komutlanip robotu DONDURUYORDU). Duz gitmek icin motorlarin TERS yonde
    komutlanmasi gerekiyor (fiziksel montaj karsilikli oldugundan), yani:
      Motor A (IN1,IN2): HIGH,LOW
      Motor B (IN3,IN4): LOW,HIGH  <-- Motor A'nin tam tersi

    Eger robot geri giderse ya da donerse, asagidaki iki satiri birbiriyle
    degistirerek dene.
    """
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)


def ileri_git_sabit_mesafe(pwm_a, pwm_b, mesafe_cm, bridge=None):
    """
    Sabit mesafe ileri gider. bridge verilirse, BNO055 heading feedback ile
    sapmayi (saga/sola kayma) anlik olarak duzeltir - bridge verilmezse
    eskisi gibi acik dongu (duzeltmesiz) calisir.
    """
    sure = mesafe_cm / CM_PER_SANIYE
    print(f"Ileri gidiliyor: {mesafe_cm} cm (~{sure:.2f}s)")

    ileri_yon_ayarla()

    if bridge is None:
        pwm_a.ChangeDutyCycle(SOL_HIZ)
        pwm_b.ChangeDutyCycle(SAG_HIZ)
        time.sleep(sure)
        motorlari_durdur(pwm_a, pwm_b)
        return

    # ---- Kapali dongu: hedef heading'i koru ----
    hedef_heading = bridge.get_heading()
    print(f"  [DEBUG] Hedef heading: {hedef_heading}")
    baslangic = time.time()
    adim_sayaci = 0

    while time.time() - baslangic < sure:
        simdiki_heading = bridge.get_heading()

        if simdiki_heading is not None and hedef_heading is not None:
            # Pozitif hata: heading artmis (robot 'sag'a donmus) -> bunu duzeltmek icin
            # sola kivrilmali -> sol tekeri YAVASLAT, sag tekeri HIZLANDIR
            # Negatif hata: heading azalmis (robot 'sol'a donmus) -> tam tersi
            hata = aci_farki(hedef_heading, simdiki_heading)
            duzeltme = max(-MAKS_DUZELTME, min(MAKS_DUZELTME, DUZELTME_KAZANCI * hata))

            sol_duty = max(0, min(100, SOL_HIZ - duzeltme))
            sag_duty = max(0, min(100, SAG_HIZ + duzeltme))

            pwm_a.ChangeDutyCycle(sol_duty)
            pwm_b.ChangeDutyCycle(sag_duty)
        else:
            sol_duty, sag_duty = SOL_HIZ, SAG_HIZ
            pwm_a.ChangeDutyCycle(SOL_HIZ)
            pwm_b.ChangeDutyCycle(SAG_HIZ)

        if adim_sayaci % 3 == 0:  # her ~150ms'de bir goster, ekrani doldurmasin
            print(f"  [DEBUG] heading={simdiki_heading} sol_duty={sol_duty:.1f} sag_duty={sag_duty:.1f}")
        adim_sayaci += 1

        time.sleep(0.05)

    motorlari_durdur(pwm_a, pwm_b)
    print(f"  [DEBUG] Ileri gitme tamamlandi, toplam {adim_sayaci} adim calisti.")


def ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=ENGEL_ESIGI_CM,
                              maks_sure=MAKS_ENGEL_BEKLEME):
    """
    Bir cisim esik_cm mesafesine kadar yaklasana kadar duz gider.
    Esige iki kademeli yavaslama uygulanir (once YAVASLAMA_MESAFE_FARKI,
    sonra COK_YAVAS_MESAFE_FARKI kala) - boylece durma ani daha hassas olur,
    overshoot (esigi fazla gecme) azalir. Ayrica BNO055 heading feedback ile
    saga/sola kaymayi (drift) anlik olarak duzeltir.

    ONEMLI: Ultrasonik sensor (HC-SR04) acili yuzeylerden/duvar koselerinden
    yansiyinca (multipath) tek seferlik, gercek olmayan yakin mesafe okumalari
    verebilir. Bunu filtrelemek icin sadece GERCEKTEN YENI gelen Arduino
    orneklerini sayiyoruz (bridge'in ic 'last_update' zaman damgasina bakarak) -
    ayni bayat degeri hizli polling yuzunden birden fazla kez okumus olmak,
    "dogrulanmis ardisik okuma" sayilmiyor.
    """
    print(f"Engel araniyor (esik: {esik_cm} cm)...")

    ileri_yon_ayarla()

    hedef_heading = bridge.get_heading()  # duz gitmeyi korumak icin referans yon

    temel_sol, temel_sag = SOL_HIZ, SAG_HIZ
    pwm_a.ChangeDutyCycle(temel_sol)
    pwm_b.ChangeDutyCycle(temel_sag)

    yavas_moda_gecildi = False
    cok_yavas_moda_gecildi = False
    ardisik_esik_alti = 0
    GEREKEN_ARDISIK_OKUMA = 3
    son_islenen_zaman_damgasi = None

    baslangic = time.time()
    while time.time() - baslangic < maks_sure:
        if bridge.is_stale(max_age_sec=0.5):
            time.sleep(0.03)
            continue

        veri = bridge.get_all()
        mesafe = veri["distance"]
        simdiki_heading = veri["heading"]
        guncel_zaman_damgasi = veri["last_update"]

        # ---- Heading feedback ile saga/sola kaymayi duzelt ----
        if simdiki_heading is not None and hedef_heading is not None:
            hata = aci_farki(hedef_heading, simdiki_heading)
            duzeltme = max(-MAKS_DUZELTME, min(MAKS_DUZELTME, DUZELTME_KAZANCI * hata))
        else:
            duzeltme = 0.0

        sol_duty = max(0, min(100, temel_sol - duzeltme))
        sag_duty = max(0, min(100, temel_sag + duzeltme))
        pwm_a.ChangeDutyCycle(sol_duty)
        pwm_b.ChangeDutyCycle(sag_duty)

        # Bu, bir onceki kontrolden beri gelen GERCEKTEN YENI bir ornek mi?
        yeni_ornek_mi = (son_islenen_zaman_damgasi is None or
                         guncel_zaman_damgasi != son_islenen_zaman_damgasi)

        if yeni_ornek_mi and mesafe is not None and mesafe > 0:  # -1 = gecersiz okuma, yoksay
            son_islenen_zaman_damgasi = guncel_zaman_damgasi
            print(f"  Mesafe: {mesafe:.1f} cm")

            # 2. kademe: esige iyice yaklasinca daha da yavasla (once kontrol
            # edilmeli, cunku 2. kademe esigi 1. kademe esiginin icinde kalir)
            if not cok_yavas_moda_gecildi and mesafe <= (esik_cm + COK_YAVAS_MESAFE_FARKI):
                temel_sol, temel_sag = HIZ_COK_YAVAS_ENGEL, HIZ_COK_YAVAS_ENGEL + (SAG_HIZ - SOL_HIZ)
                cok_yavas_moda_gecildi = True
                yavas_moda_gecildi = True  # 1. kademeyi de gecmis sayilir
                print(f"  (Son yaklasim - iyice yavaslandi: {mesafe:.1f} cm)")
            # 1. kademe: esige yaklasinca yavasla
            elif not yavas_moda_gecildi and mesafe <= (esik_cm + YAVASLAMA_MESAFE_FARKI):
                temel_sol, temel_sag = HIZ_YAVAS_ENGEL, HIZ_YAVAS_ENGEL + (SAG_HIZ - SOL_HIZ)
                yavas_moda_gecildi = True
                print(f"  (Yavaslama bolgesine girildi: {mesafe:.1f} cm)")

            if mesafe <= esik_cm:
                ardisik_esik_alti += 1
                if ardisik_esik_alti >= GEREKEN_ARDISIK_OKUMA:
                    motorlari_durdur(pwm_a, pwm_b)
                    print(f"Engel tespit edildi ({mesafe:.1f} cm), duruldu.")
                    return True
            else:
                ardisik_esik_alti = 0

        time.sleep(0.03)  # daha sik ornekleme, tepki suresini kisaltir

    motorlari_durdur(pwm_a, pwm_b)
    print("UYARI: Zaman asimi, engel bulunamadi. Guvenlik amacli durduruldu.")
    return False


def guvenli_donus(hedef_derece, yon, bridge, pwm_a, pwm_b, maks_deneme=2):
    """
    donus_yap()'i cagirir, ama sonucu KONTROL EDER. Eger donus BNO055
    kilitlenmesi ya da baska bir sorun yuzunden basarisiz olduysa (donen
    aci, hedefin cok altindaysa - orn. yarisindan az), bunu sessizce
    gecmez: uyari basar ve tekrar dener. Tum denemeler basarisiz olursa,
    False donup testin guvenli sekilde durmasini saglar.

    Donus deger: True (basarili) / False (tum denemeler basarisiz)
    """
    for deneme in range(1, maks_deneme + 1):
        sonuc = donus_yap(hedef_derece, yon=yon, bridge=bridge, pwm_a=pwm_a, pwm_b=pwm_b)

        # Basari kontrolu: donen aci, hedefin en az yarisi kadar olmali.
        # Kilitlenme durumunda donus_yap 0.0 (ya da cok kucuk bir deger) doner.
        if abs(sonuc) >= hedef_derece * 0.5:
            return True

        print(f"\nUYARI: Donus basarisiz gorunuyor (istenen {hedef_derece}, "
              f"gerceklesen {sonuc:.1f}). Deneme {deneme}/{maks_deneme}.")

        if deneme < maks_deneme:
            print("Tekrar deneniyor...\n")
            time.sleep(1.0)  # sensorun toparlanmasi icin kisa bir bekleme

    print(f"\nHATA: {maks_deneme} denemede de donus basarili olamadi. "
          f"Test guvenlik amacli durduruluyor.")
    print("Onerilen adimlar: BNO055 baglantisini/montajini kontrol et, "
          "titresim/EMI kaynaklarini gozden gecir.")
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

    # PWM nesnelerini BIR KEZ olustur, tum test boyunca (donus + duz gitme)
    # ayni nesneleri kullan. RPi.GPIO'nun ayni pinde ust uste PWM
    # olusturma/yikma sirasinda bazen motoru tepkisiz birakmasi sorununu onler.
    pwm_a, pwm_b = motorlari_ayarla()

    try:
        # ---- 1) Engele kadar duz git ----
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b)

        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        # ---- 2) 90 derece sola don (BNO055 kapali dongu) ----
        print("\n=== 1. DONUS: 90 derece sola ===")
        if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (1. donus basarisiz).")
            return

        # ---- 3) Engele (30 cm) kadar duz git ----
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b)
        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        # ---- 4) Tekrar 90 derece sola don ----
        print("\n=== 2. DONUS: 90 derece sola ===")
        if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
            print("Test durduruldu (2. donus basarisiz).")
            return

        # ---- 5) Engele (30 cm) kadar duz git ----
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b)
        if not bulundu:
            print("Engel bulunamadigi icin test durduruldu.")
            return

        print("\nTest surusu tamamlandi.")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
