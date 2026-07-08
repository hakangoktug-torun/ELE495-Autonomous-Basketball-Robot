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
from robot_bridge import RobotBridge

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
FINE_TOLERANS = 1.0        # bu derecenin altindaki hata artik kabul edilir (3.0'dan dusuruldu - deneme)
DUZELTME_HIZ = 30           # HIZ_NORMAL ile ayni - kisa atislarda dusuk duty tekerlegi hic hareket ettirmiyor
DUZELTME_MIN_SURE = 0.08    # saniye - motorun baslama gecikmesini (spin-up) guvenle asacak minimum sure
DUZELTME_MAX_SURE = 0.15    # saniye - en uzun duzeltme atisi
DUZELTME_SETTLE = 0.4       # her atistan sonra olcum oncesi bekleme (magnetometer/motor sakinlessin)
MAKS_DUZELTME_DENEME = 12   # sonsuz salinim olmasin diye deneme sinirI


def kalibrasyon_bekle(bridge, hedef_sys=3, hedef_gyro=3, hedef_accel=1, hedef_mag=3,
                        kontrol_araligi=2.0):
    """
    Rotasyon testi baslamadan once BNO055'in parametrelerinin hedef seviyeye
    ulasmasini bekler. hedef_accel varsayilan olarak 1 - cunku sasiye monteli
    bir sensoru elle 6 farkli yonde tam sabit tutmak pratikte cok zor, ve
    Bosch'un kendi fuzyon algoritmasi bile sys=3'e accel=1 ile ulasabiliyor
    (yani tam accel=3 sart degil, gyro ve mag cok daha kritik).
    Her dongude, hangi parametrenin eksik oldugunu ve ne yapman gerektigini
    ekrana yazdirir. sys genelde digerleri tamamlaninca kendiliginden yukselir.
    """
    print("\n=== KALIBRASYON KONTROLU ===")

    # Once EEPROM'dan kayitli kalibrasyon yuklenmis mi diye bak. Yuklendiyse,
    # canli 'sys' degeri dusuk gorunse bile (BNO055'in durum bitleri her
    # acilista sifirlanir, ama offset'ler zaten yuklenip kullanilmaya
    # BASLAMISTIR) uzun uzun beklemek yerine kisa bir onay sorup gecebiliriz.
    bridge.request_calibration_status()
    time.sleep(0.3)
    cal = bridge.get_calibration()

    if cal.get("eeprom_yuklendi"):
        print("Arduino'da onceden kaydedilmis bir kalibrasyon yuklu.")
        print("(NOT: sys/gyro/accel/mag degerleri her acilista sifirlanir, "
              "ama offset'ler zaten arka planda kullaniliyor olabilir.)")
        cevap = input("Kayitli kalibrasyonla devam edilsin mi? (devam icin 'yes' yaz, "
                       "sifirdan kalibrasyon icin Enter'a bas): ").strip().lower()
        if cevap == "yes":
            print("Kayitli kalibrasyonla devam ediliyor.\n")
            return True
        print()  # kullanici sifirdan kalibrasyon istedi, asagidaki donguye devam

    print("Her parametre 3'e (mukemmel) ulasana kadar bekleniyor.\n")

    while True:
        bridge.request_calibration_status()
        time.sleep(0.3)  # cevabin _read_loop icinde islenmesini bekle

        cal = bridge.get_calibration()
        sys_v = cal["sys"] if cal["sys"] is not None else 0
        gyro_v = cal["gyro"] if cal["gyro"] is not None else 0
        accel_v = cal["accel"] if cal["accel"] is not None else 0
        mag_v = cal["mag"] if cal["mag"] is not None else 0

        print(f"Guncel kalibrasyon -> sys={sys_v} gyro={gyro_v} "
              f"accel={accel_v} mag={mag_v}  (0=kotu, 3=mukemmel)")

        mesafe = bridge.get_distance()
        r, g, b, c = bridge.get_color()
        ir1, ir2 = bridge.get_ir()
        print(f"  Mesafe: {mesafe} cm | Renk (R,G,B,C): ({r},{g},{b},{c}) | IR: ({ir1},{ir2})")

        eksikler = []
        if gyro_v < hedef_gyro:
            eksikler.append("  - GYRO: Sensoru (robotu) birkac saniye TAMAMEN SABIT tut, "
                             "duz bir yuzeye koyup dokunma.")
        if accel_v < hedef_accel:
            eksikler.append("  - ACCEL: Sensoru sirayla farkli yonlerde (duz, ters, sag yan, "
                             "sol yan, on egik, arka egik) birkac saniye sabit tutarak cevir.")
        if mag_v < hedef_mag:
            eksikler.append("  - MAG: Havada '8' cizer gibi yavasca, farkli yonlerde cevir.")

        if sys_v >= hedef_sys:
            print(f"\nsys={sys_v} (hedefe ulasti).")
            cevap = input("Donus testine baslansin mi? (baslamak icin 'yes' yaz, "
                           "beklemeye devam icin Enter'a bas): ").strip().lower()
            if cevap == "yes":
                print("Devam ediliyor.\n")
                return True
            print()  # beklemeye devam et, dongu basa doner

        if eksikler:
            print("Yapman gerekenler:")
            for satir in eksikler:
                print(satir)
        else:
            print("  (gyro/accel/mag tamam, sys'in kendiliginden yukselmesi bekleniyor...)")

        print()
        time.sleep(kontrol_araligi)


def aci_farki(baslangic, bitis):
    """0-360 derece sarmalini dogru hesaplayan aci farki (-180, +180] araliginda."""
    fark = bitis - baslangic
    if fark < -180:
        fark += 360
    elif fark > 180:
        fark -= 360
    return fark


def guvenli_heading_guncelle(onceki_heading, yeni_heading, bekleyen_deger, maks_adim, sessiz=False):
    """
    Yeni bir heading okumasini kumulatif toplama eklerken supheli buyuk
    sicramalari filtreler. Eger sicrama bir sonraki ornekte tekrar ederse
    (5 derece toleransla) gercek kabul edilir, etmezse yoksayilir.

    Donus deger: (eklenecek_delta, guncel_onceki_heading, guncel_bekleyen_deger)
    eklenecek_delta None ise, bu ornek hicbir sekilde eklenmemeli.
    """
    if onceki_heading is None or yeni_heading is None:
        return None, onceki_heading, bekleyen_deger

    adim = aci_farki(onceki_heading, yeni_heading)

    if abs(adim) <= maks_adim:
        return adim, yeni_heading, None

    # Buyuk sicrama - hemen kabul etme
    if bekleyen_deger is not None and abs(aci_farki(bekleyen_deger, yeni_heading)) <= 5.0:
        if not sessiz:
            print(f"  (Sicrama dogrulandi, gercek kabul edildi: {adim:.1f} derece)")
        return adim, yeni_heading, None

    if not sessiz:
        print(f"  (Supheli sicrama gorzmezden gelindi: {adim:.1f} derece)")
    return None, onceki_heading, yeni_heading


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
    MAKS_GECERLI_DUZELTME_ADIMI = 30.0  # derece - kisa bir atista bundan buyugu supheli sayilir
    bekleyen_deger = None
    ARDISIK_DEGISMEME_LIMITI = 3  # bu kadar atis ust uste heading'i hic degistirmezse kilitlenme say
    ardisik_degismeyen_sayisi = 0

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
        heading_oncesi = onceki_heading
        delta, onceki_heading, bekleyen_deger = guvenli_heading_guncelle(
            onceki_heading, yeni_heading, bekleyen_deger, MAKS_GECERLI_DUZELTME_ADIMI
        )
        if delta is not None:
            toplam_donus += delta
            ardisik_degismeyen_sayisi = 0
        else:
            ardisik_degismeyen_sayisi += 1
            if ardisik_degismeyen_sayisi >= ARDISIK_DEGISMEME_LIMITI:
                print(f"UYARI: BNO055 KILITLENMIS gorunuyor - {ARDISIK_DEGISMEME_LIMITI} atis "
                      f"boyunca heading hic degismedi. Sensor resetleniyor, duzeltme iptal ediliyor.")
                bridge.request_heading_reset()
                time.sleep(1.0)
                return toplam_donus, onceki_heading

    hata_isaretli = hedef_isaretli - toplam_donus
    print(f"UYARI: Maksimum duzeltme denemesi ({MAKS_DUZELTME_DENEME}) doldu. "
          f"Kalan hata: {abs(hata_isaretli):.2f} derece (hedeflenen: {FINE_TOLERANS} derece)")
    return toplam_donus, onceki_heading


def donus_yap(hedef_derece, yon="sol", bridge=None, pwm_a=None, pwm_b=None):
    """
    hedef_derece: kac derece donulecek (pozitif sayi, yon parametresi ile yon belirlenir)
    yon: 'sol' ya da 'sag'
    bridge: disaridan RobotBridge nesnesi verilebilir (main() disinda cagrilirsa)
    pwm_a, pwm_b: disaridan PWM nesneleri verilebilir. Verilirse, bu fonksiyon
        kendi GPIO/PWM kurulumunu yapmaz ve sonunda GPIO.cleanup() CAGIRMAZ -
        boylece ust seviye kod (ornegin test_surus.py) PWM nesnelerini tekrar
        tekrar kurup yikmadan, ayni nesneleri donus + duz gitme arasinda
        kesintisiz kullanabilir (RPi.GPIO'nun ayni pinde ust uste PWM
        olusturma/yikma sirasinda bazen motoru tepkisiz birakmasi sorununu onler).

    Donus deger: gercekte donulen toplam derece (float)
    """
    kendi_bridge = False
    if bridge is None:
        bridge = RobotBridge(port=SERIAL_PORT)
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

    kendi_motor = pwm_a is None or pwm_b is None
    if kendi_motor:
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
        son_gecerli_veri_zamani = time.time()
        bekleyen_deger_ana = None
        MAKS_GECERLI_ANA_ADIM = 30.0  # derece - 20ms'de bundan buyuk gercek olamaz (motor bu kadar hizli donemez)

        # BNO055 KILITLENME tespiti: veri akisi devam etse bile (is_stale=False),
        # eger heading DEGERI belirli bir sure hic degismezse (motor aktif
        # donerken), bu sensorun kilitlenip ayni degeri tekrarladigi anlamina
        # gelir - bilinen bir BNO055 firmware davranisi. Bu, is_stale()'den
        # FARKLI bir kontrol: is_stale sadece veri akisinin ZAMANLAMASINA bakar,
        # burada ise verinin ICERIGININ degisip degismedigine bakiyoruz.
        KILITLENME_ESIGI = 0.5  # saniye - bu sure boyunca heading hic degismezse kilitlenme say
                                  # (kisa tutuldu: motor tam hizdayken uzun esik = fazla kontrolsuz donus)
        DEGISIM_EPSILON = 0.3   # derece - bunun altindaki farklar "degismedi" sayilir
        son_degisim_zamani = time.time()
        son_bilinen_deger = onceki_heading

        while True:
            if time.time() - baslangic_zamani > ZAMAN_ASIMI:
                print("UYARI: Zaman asimi, donus zorla durduruldu (sensor/motor sorunu olabilir).")
                break

            # Baglanti titresim/motor yuzunden kesildiyse hemen dur - korlemesine
            # 8 saniye boyunca donmeye devam etmesin.
            if bridge.is_stale(max_age_sec=0.5):
                if time.time() - son_gecerli_veri_zamani > 0.5:
                    print("UYARI: BNO055 veri akisi kesildi (muhtemelen titresim/breadboard "
                          "baglanti sorunu). Donus guvenlik amacli hemen durduruldu.")
                    break
            else:
                son_gecerli_veri_zamani = time.time()

            simdiki_heading = bridge.get_heading()

            # KILITLENME kontrolu - veri akiyor ama deger hic degismiyor mu?
            if simdiki_heading is not None:
                if son_bilinen_deger is None or abs(aci_farki(son_bilinen_deger, simdiki_heading)) > DEGISIM_EPSILON:
                    son_bilinen_deger = simdiki_heading
                    son_degisim_zamani = time.time()
                elif time.time() - son_degisim_zamani > KILITLENME_ESIGI:
                    print(f"UYARI: BNO055 KILITLENMIS gorunuyor - heading {KILITLENME_ESIGI}s'den "
                          f"uzun suredir hic degismedi ({simdiki_heading}), motor aktif donmesine ragmen. "
                          f"Sensor resetleniyor ve donus iptal ediliyor.")
                    motorlari_durdur(pwm_a, pwm_b)
                    bridge.request_heading_reset()
                    time.sleep(1.0)
                    return 0.0

            delta, onceki_heading, bekleyen_deger_ana = guvenli_heading_guncelle(
                onceki_heading, simdiki_heading, bekleyen_deger_ana, MAKS_GECERLI_ANA_ADIM
            )
            if delta is not None:
                toplam_donus += delta

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

        # Motor calisirken olusan manyetik girisimin sonmesi icin bekle.
        # ONEMLI: Bu sirada motor KAPALI, yani robot fiziksel olarak hareket etmiyor.
        # Bu yuzden bu pencerede gorulen buyuk bir ani sicrama gercek bir donus OLAMAZ -
        # sensorun toparlanma surecindeki gecici bir gurultu/glitch'tir. Tek bir
        # once/sonra olcumu yerine surekli orneklyip, supheli buyuk sicramalari
        # reddederek (ama ayni deger tekrar ederse gercek kabul ederek) daha
        # guvenilir bir sonuc elde ediyoruz.
        SETTLE_SURESI = 1.5
        SETTLE_ORNEK_ARALIGI = 0.1
        MAKS_GECERLI_SETTLE_ADIMI = 15.0  # derece - motor kapaliyken bundan buyugu supheli sayilir

        print(f"Motor durdu, magnetometer'in sakinlesmesi icin {SETTLE_SURESI}s bekleniyor "
              f"(supheli sicramalar filtreleniyor)...")

        bekleyen_deger_settle = None

        settle_baslangic = time.time()
        while time.time() - settle_baslangic < SETTLE_SURESI:
            simdiki_heading = bridge.get_heading()
            delta, onceki_heading, bekleyen_deger_settle = guvenli_heading_guncelle(
                onceki_heading, simdiki_heading, bekleyen_deger_settle, MAKS_GECERLI_SETTLE_ADIMI
            )
            if delta is not None:
                toplam_donus += delta

            time.sleep(SETTLE_ORNEK_ARALIGI)

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
        if kendi_motor:
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
