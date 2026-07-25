"""
Kapali dongu (closed-loop) donus kontrolu - BNO055 feedback ile
Sabit sure yerine, hedef aciya ulasana kadar doner ve otomatik durur.

GUNCELLEME (acil durdur destegi): donus_yap() ve ince_duzeltme_yap() artik
opsiyonel bir dur_bayragi (threading.Event) parametresi aliyor. Bu bayrak
set edildiginde (Acil Durdur butonu ya da Ctrl+C), donus dongusu EN GEC bir
sonraki kontrol turunda (birkac-birkac yuz ms icinde) motoru durdurup
fonksiyondan hemen cikiyor.

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
HIZ_NORMAL = 45      # ana donus hizi (30'dan 45'ye - 1.5x, sag donuste asma/sag arka
                       # teker tutuklugu devam ettigi icin 1.4x'ten yukseltildi)
HIZ_YAVAS = 33        # hedefe yaklasirken yavaslama hizi (31'den 33'e - 1.5x)
YAVASLAMA_ESIGI = 45.0  # hedefe kalan derece bu esigin altina dusunce yavasla (1. kademe) (42'den 45'e - 1.5x)
COK_YAVAS_ESIGI_DONUS = 12.0  # hedefe kalan derece bu esigin altina dusunce IYICE yavasla (2. kademe) (11'den 12'ye - 1.5x)
HIZ_COK_YAVAS_DONUS = 27     # 2. kademe hizi (25'ten 27'ye - 1.5x)
TOLERANS = 2.0         # hedefe bu kadar derece yakinsa "ulasti" say
ZAMAN_ASIMI = 8.0      # saniye - sensor/motor sorununda sonsuz donmeyi engeller

SERIAL_PORT = "/dev/ttyUSB0"

# ---------- Ince duzeltme (fine correction) ayarlari ----------
FINE_TOLERANS = 1.0        # bu derecenin altindaki hata artik kabul edilir. (2.0'dan 1.0'a -
                             # son testlerde atis basina ~1-2 derecelik ilerleme gozlemlendi,
                             # yani kuantum artik yeterince ince - 1.0 hedefi artik guvenli.)
DUZELTME_HIZ = 45           # HIZ_NORMAL ile ayni (1.5x) - kisa atislarda dusuk duty tekerlegi hic hareket ettirmiyor
DUZELTME_MIN_SURE = 0.053   # saniye - (0.057'den 0.053'e - DUZELTME_HIZ 1.5x arttigi icin orantili kisaltildi)
DUZELTME_MIN_SURE_INCE = 0.03  # saniye - hata kucukken (asagidaki INCE_ESIGI altinda) kullanilan
                                 # DAHA KISA atis suresi - 1 derece hedefe yaklasirken normal minimum
                                 # sure bile fazla agresif kalabiliyor, bu daha ince bir adim saglar
INCE_ESIGI = 3.0            # derece - hata bu esigin altindaysa DUZELTME_MIN_SURE_INCE kullanilir
DUZELTME_MAX_SURE = 0.10    # saniye - en uzun duzeltme atisi (0.107'den 0.10'a - 1.5x)
DUZELTME_SETTLE = 0.4       # her atistan sonra olcum oncesi bekleme (magnetometer/motor sakinlessin)
MAKS_DUZELTME_DENEME = 10   # sonsuz salinim olmasin diye deneme siniri (8'den 10'a - FINE_TOLERANS
                              # sikilastigi icin (1.0) bazen 1-2 atis daha gerekebilir)


def _durdurma_istendi_mi(dur_bayragi):
    """Ortak yardimci - dur_bayragi verilmis ve set edilmisse True doner."""
    return dur_bayragi is not None and dur_bayragi.is_set()


def kalibrasyon_bekle(bridge, hedef_sys=3, hedef_gyro=3, hedef_accel=1, hedef_mag=3,
                        kontrol_araligi=2.0, otonom=False, otonom_maks_bekleme=6.0):
    """
    Rotasyon testi baslamadan once BNO055'in parametrelerinin hedef seviyeye
    ulasmasini bekler. hedef_accel varsayilan olarak 1 - cunku sasiye monteli
    bir sensoru elle 6 farkli yonde tam sabit tutmak pratikte cok zor, ve
    Bosch'un kendi fuzyon algoritmasi bile sys=3'e accel=1 ile ulasabiliyor
    (yani tam accel=3 sart degil, gyro ve mag cok daha kritik).

    otonom=True ise: HICBIR klavye girisi (input()) beklenmez - demo sirasinda
    robotu elle durdurup onaylamak mumkun olmadigi icin bu mod gerekli.
    EEPROM'dan kalibrasyon yuklendiyse hemen devam eder; yuklenmediyse en fazla
    otonom_maks_bekleme saniye sys=3'u bekler, sonra ne durumda olursa olsun
    devam eder (sonsuza kadar beklemez, demo robotu kilitlenmez).

    GUNCELLEME (BUG DUZELTMESI - "bagimsiz testte duzgun, sweep akisinda
    duzensiz/hizli donuyor" sorunu): Otonom modda EEPROM kalibrasyonu
    yukluyse eskiden HICBIR CANLI DOGRULAMA yapilmadan aninda donuluyordu -
    ama Arduino'nun raporladigi sys/gyro/accel/mag degerleri HER ACILISTA
    sifirlanir (yorum yukarida), yani offsetler yuklu olsa bile fuzyon
    algoritmasinin bunlari GERCEKTEN kullanip kullanmadigi/dogru sonuc
    verip vermedigi canli olarak TEYIT EDILMIYORDU. Su an EEPROM+otonom
    durumunda bile KISA (EEPROM_DOGRULAMA_SURESI, 2s) ve SINIRLI bir canli
    kontrol yapiliyor - gyro hedefe ulasirsa hemen devam edilir, ulasmazsa
    (offsetler bayat/uyumsuz olabilir) acikca UYARILIP yine de devam edilir
    (otonom ruhuna uygun - sinirsiz beklemiyor). Ayrica otonom dongude
    (EEPROM olmayan durum) artik SADECE sys degil GYRO da kontrol ediliyor -
    donus dogrulugunu en cok etkileyen alt-sistem gyro oldugu icin.
    """
    print("\n=== KALIBRASYON KONTROLU ===")

    bridge.request_calibration_status()
    time.sleep(0.3)
    cal = bridge.get_calibration()

    if cal.get("eeprom_yuklendi"):
        if otonom:
            EEPROM_DOGRULAMA_SURESI = 2.0  # saniye - EEPROM offsetlerinin gercekten
                                             # ise yaradigini KISACA teyit etmek icin
            print(f"EEPROM'dan kalibrasyon yuklu - otonom modda kisa bir canli "
                  f"dogrulama yapiliyor (en fazla {EEPROM_DOGRULAMA_SURESI}s)...")
            baslangic = time.time()
            while time.time() - baslangic < EEPROM_DOGRULAMA_SURESI:
                bridge.request_calibration_status()
                time.sleep(0.2)
                cal = bridge.get_calibration()
                gyro_v = cal["gyro"] if cal["gyro"] is not None else 0
                if gyro_v >= hedef_gyro:
                    print(f"  gyro={gyro_v} - EEPROM offsetleri dogrulandi, devam ediliyor.\n")
                    return True
            gyro_son = cal.get("gyro")
            print(f"  UYARI: {EEPROM_DOGRULAMA_SURESI}s icinde gyro hedefe ulasmadi "
                  f"(son deger: {gyro_son}) - EEPROM offsetleri bayat/uyumsuz olabilir. "
                  f"Yine de otonom modda devam ediliyor, ama donus dogrulugu "
                  f"dusuk olabilir.\n")
            return True
        print("Arduino'da onceden kaydedilmis bir kalibrasyon yuklu.")
        print("(NOT: sys/gyro/accel/mag degerleri her acilista sifirlanir, "
              "ama offset'ler zaten arka planda kullaniliyor olabilir.)")
        cevap = input("Kayitli kalibrasyonla devam edilsin mi? (devam icin 'yes' yaz, "
                       "sifirdan kalibrasyon icin Enter'a bas): ").strip().lower()
        if cevap == "yes":
            print("Kayitli kalibrasyonla devam ediliyor.\n")
            return True
        print()  # kullanici sifirdan kalibrasyon istedi, asagidaki donguye devam

    if otonom:
        print(f"Otonom modda kalibrasyon bekleniyor (en fazla {otonom_maks_bekleme}s)...")
        baslangic = time.time()
        while time.time() - baslangic < otonom_maks_bekleme:
            bridge.request_calibration_status()
            time.sleep(0.3)
            cal = bridge.get_calibration()
            sys_v = cal["sys"] if cal["sys"] is not None else 0
            gyro_v = cal["gyro"] if cal["gyro"] is not None else 0
            if sys_v >= hedef_sys and gyro_v >= hedef_gyro:
                print(f"sys={sys_v} gyro={gyro_v} - otonom modda devam ediliyor.\n")
                return True
            time.sleep(0.5)
        print(f"UYARI: Otonom bekleme suresi doldu (sys={cal.get('sys')}, "
              f"gyro={cal.get('gyro')}), mevcut durumla devam ediliyor. "
              f"Sonuclar daha az hassas olabilir.\n")
        return True

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
    """
    Motorlari durdurur (coast - gucu keser, aktif frenleme yapmaz).

    NOT: Once burada 'aktif frenleme' (IN1=IN2=HIGH kisa devre) denedik,
    overshoot'u azaltmasi beklentisiyle. Ama (1) beklenen faydayi saglamadi
    (Ana donus sonucu hala 6-9 derece fazla cikiyordu) ve (2) sik tekrarlanan
    GPIO/PWM degisimi RPi.GPIO'nun yazilimsal PWM'inde kararsizliga yol acip
    bazen bir motor kanalinin frende 'takili' kalmasina (tek taraflarin
    calismasi, robotun yerinde donmek yerine kaymasi) sebep oldu. Basit
    coast'a geri donuldu.
    """
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


def ince_duzeltme_yap(bridge, pwm_a, pwm_b, hedef_isaretli, toplam_donus, onceki_heading,
                       dur_bayragi=None):
    """
    Ana donus bittikten sonra, kalan hata FINE_TOLERANS'in ustundeyse
    kisa duzeltme atislari (pulse) yaparak hatayi azaltmaya calisir.
    Her atistan sonra durup olcum alir, gerekirse ters yonde tekrar dener.

    GUNCELLEME (acil durdur): dur_bayragi set edilirse, dongu bir sonraki
    denemeye gecmeden once motoru durdurup mevcut degerlerle hemen doner.

    ADAPTIF ESKALASYON: Eger art arda atislar hatada anlamli bir ilerleme
    saglamiyorsa (robot belirli bir direncli/stall konumunda takili
    kaliyorsa), atis suresini kademeli olarak uzatarak bu direnci
    kirmaya calisir - sabit minimum sure (DUZELTME_MIN_SURE) bazen
    yetersiz kalabiliyor. Eskalasyon 2.5x ile SINIRLI - daha fazlasi,
    stall'dan kurtulunca asiri buyuk bir harekete (overshoot) yol acip
    hatayi BASLANGICTAN DAHA KOTU hale getirebiliyor (gozlemlendi).

    EN IYI SONUC GUVENLIK AGI: Fonksiyon, gordugu TUM denemeler arasindan
    en dusuk |hata| degerine sahip olani hatirlar ve donus ne sekilde
    biterse bitsin (limit dolsun, kilitlenme olsun, salinim olsun) HER
    ZAMAN en iyi gorulen sonucu dondurur - boylece son atisin kotu gitmesi
    yuzunden zaten iyi olan bir ara sonucun kaybedilmesi engellenir.

    Donus deger: (guncellenmis toplam_donus, guncellenmis onceki_heading)
    """
    MAKS_GECERLI_DUZELTME_ADIMI = 30.0  # derece - kisa bir atista bundan buyugu supheli sayilir
    bekleyen_deger = None
    ARDISIK_DEGISMEME_LIMITI = 3  # bu kadar atis ust uste heading'i hic degistirmezse kilitlenme say
    ardisik_degismeyen_sayisi = 0

    MIN_ANLAMLI_ILERLEME = 0.5  # derece - bir atisin 'ise yaradi' sayilmasi icin gereken en az iyilesme
    ILERLEMESIZ_ESKALASYON_ESIGI = 2  # bu kadar atis ust uste yeterli ilerleme saglamazsa sureyi uzat
    ESKALASYON_CARPANI = 1.5  # her eskalasyonda sure kac katina cikar
    ESKALASYON_TAVANI = 2.5  # MUTLAK ust sinir - eskiden 4.38x'e kadar cikiyordu, bu asiri overshoot'a
                               # yol aciyordu (robot fiziksel olarak baslangictan daha kotu bir konumda
                               # kalabiliyordu). 2.5x ile sinirlandirildi - overshoot riski azaltilir.
    ilerlemesiz_sayisi = 0
    eskalasyon_carpani = 1.0
    onceki_hata_buyuklugu = None

    # NOT: Onceden 'en iyi sonucu hatirlayip donme' yaklasimi denendi, ama bu
    # YANLIS - robot FIZIKSEL olarak son atisin biraktigi yerde kalir, sadece
    # bir sayi degistirerek 'daha iyi bir konumda' oldugumuzu iddia etmek
    # sisteme yalan soylemek olurdu (sonraki kodlar gercek olmayan bir
    # referanstan calisir). Bunun yerine MAKS_DUZELTME_DENEME'yi biraz
    # artirip, eskalasyonu sinirlayarak robotun GERCEKTEN kendini
    # duzeltebilmesi icin daha fazla sansi ayni dongu icinde tanıyoruz.

    for deneme in range(1, MAKS_DUZELTME_DENEME + 1):
        if _durdurma_istendi_mi(dur_bayragi):
            motorlari_durdur(pwm_a, pwm_b)
            print("DURDURMA sinyali alindi - ince duzeltme aninda iptal ediliyor.")
            return toplam_donus, onceki_heading

        hata_isaretli = hedef_isaretli - toplam_donus

        if abs(hata_isaretli) <= FINE_TOLERANS:
            print(f"Ince duzeltme tamamlandi ({deneme - 1} atis sonrasi). "
                  f"Kalan hata: {abs(hata_isaretli):.2f} derece")
            return toplam_donus, onceki_heading

        # Bir onceki atisin gercekten ise yarayip yaramadigini kontrol et
        if onceki_hata_buyuklugu is not None:
            ilerleme = onceki_hata_buyuklugu - abs(hata_isaretli)
            if ilerleme < MIN_ANLAMLI_ILERLEME:
                ilerlemesiz_sayisi += 1
            else:
                ilerlemesiz_sayisi = 0
                eskalasyon_carpani = 1.0  # gercek ilerleme oldu, eskalasyonu sifirla

        if ilerlemesiz_sayisi >= ILERLEMESIZ_ESKALASYON_ESIGI:
            eskalasyon_carpani = min(eskalasyon_carpani * ESKALASYON_CARPANI, ESKALASYON_TAVANI)
            print(f"  (Ilerleme yok - atis suresi eskalasyon carpani: {eskalasyon_carpani:.2f}x)")

        onceki_hata_buyuklugu = abs(hata_isaretli)

        yon_bu_atis = "sag" if hata_isaretli > 0 else "sol"
        if abs(hata_isaretli) < INCE_ESIGI:
            # Hata zaten kucuk - normal minimum sure bile fazla agresif kalabilir,
            # daha ince bir atis suresi kullan (1 derece hedefe daha guvenli yaklasim).
            temel_pulse_sure = DUZELTME_MIN_SURE_INCE
        else:
            temel_pulse_sure = min(DUZELTME_MAX_SURE, max(DUZELTME_MIN_SURE, abs(hata_isaretli) / 200.0))
        pulse_sure = min(DUZELTME_MAX_SURE * ESKALASYON_TAVANI, temel_pulse_sure * eskalasyon_carpani)

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


def isinma_yap(bridge, pwm_a, pwm_b, hiz=30):
    """
    Demo sirasinda robotu elle kalibre etmek mumkun olmadigi icin, robotun
    KENDI motorlariyla kucuk bir 'sallanma' hareketi yaptirarak BNO055'in
    fusion algoritmasinin canli veriyle toparlanmasina yardimci olur.

    NOT: Bu, elle '8 cizme' hareketinin YERINE GECMEZ - duz zeminde hareket
    eden bir robot sadece yaw ekseninde donebiliyor, mag kalibrasyonunun
    tam 3'e ulasmasi icin gereken 3 boyutlu egik hareketleri yapamaz. Bu
    fonksiyon sadece fusion algoritmasinin EEPROM'dan yuklenen offset'lerle
    birlikte 'canli veriyle oturmasina' yardimci olur - mag=3 garantisi vermez.

    ONEMLI: Onceki versiyon acik dongu (zamanlamaya guvenen) sallanma
    yapiyordu - motorlar arasindaki asimetri yuzunden bu, robotun BASLADIGI
    yone tam donmemesine (kalici bir yon kaymasina) yol aciyordu. Simdi
    sallanmadan once/sonra heading OLCULUYOR, ve gerekirse kucuk bir
    duzeltme atisiyla robot baslangic yonune (yaklasik olarak) geri
    getiriliyor.
    """
    print("Otomatik isinma hareketi yapiliyor (elle kalibrasyon yerine)...")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.setup(p, GPIO.OUT)

    baslangic_heading = bridge.get_heading()

    # Kucuk sag-sol-sag sallanma: her adim ~10-15 derece kadar surer
    adimlar = [("sag", 0.15), ("sol", 0.30), ("sag", 0.15)]

    for yon, sure in adimlar:
        donus_yonu_ayarla(yon)
        pwm_a.ChangeDutyCycle(hiz)
        pwm_b.ChangeDutyCycle(hiz)
        time.sleep(sure)
        motorlari_durdur(pwm_a, pwm_b)
        time.sleep(0.2)

    # Sallanma bittikten sonra sensorun toparlanmasi icin kisa bir bekleme
    time.sleep(0.5)

    # ---- Baslangic yonune donme kontrolu (tekrarli - tek atis yeterli olmayabilir) ----
    MAKS_ISINMA_DUZELTME_DENEME = 3
    for isinma_deneme in range(1, MAKS_ISINMA_DUZELTME_DENEME + 1):
        simdiki_heading = bridge.get_heading()
        if baslangic_heading is None or simdiki_heading is None:
            break

        yon_kaymasi = aci_farki(baslangic_heading, simdiki_heading)
        if abs(yon_kaymasi) <= 3.0:
            if isinma_deneme == 1:
                print(f"  Isinma sonrasi yon kaymasi kabul edilebilir seviyede: {yon_kaymasi:.1f} derece")
            else:
                print(f"  Duzeltme sonrasi kalan yon kaymasi kabul edilebilir: {yon_kaymasi:.1f} derece")
            break

        print(f"  Yon kaymasi tespit edildi: {yon_kaymasi:.1f} derece, "
              f"duzeltme deneniyor ({isinma_deneme}/{MAKS_ISINMA_DUZELTME_DENEME})...")
        duzeltme_yonu = "sol" if yon_kaymasi > 0 else "sag"
        duzeltme_suresi = min(0.25, max(0.04, abs(yon_kaymasi) / 100.0))
        donus_yonu_ayarla(duzeltme_yonu)
        pwm_a.ChangeDutyCycle(hiz)
        pwm_b.ChangeDutyCycle(hiz)
        time.sleep(duzeltme_suresi)
        motorlari_durdur(pwm_a, pwm_b)
        time.sleep(0.3)
    else:
        son_heading = bridge.get_heading()
        if baslangic_heading is not None and son_heading is not None:
            kalan_kayma = aci_farki(baslangic_heading, son_heading)
            print(f"  UYARI: {MAKS_ISINMA_DUZELTME_DENEME} denemede tam duzeltilemedi. "
                  f"Kalan kayma: {kalan_kayma:.1f} derece")

    bridge.request_calibration_status()
    time.sleep(0.3)
    cal = bridge.get_calibration()
    print(f"Isinma sonrasi kalibrasyon -> sys={cal['sys']} gyro={cal['gyro']} "
          f"accel={cal['accel']} mag={cal['mag']}\n")


def donus_yap(hedef_derece, yon="sol", bridge=None, pwm_a=None, pwm_b=None, otonom=False,
              dur_bayragi=None):
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
    dur_bayragi: (YENI) opsiyonel threading.Event. Set edilirse, donus dongusu
        EN GEC bir sonraki kontrol turunda (yaklasik 20ms) motoru durdurup
        fonksiyondan hemen cikar - Acil Durdur / Ctrl+C icin kullanilir.

    GUNCELLEME (OVERSHOOT AZALTMA - aktif fren darbesi): Motor, "kalan <=
    TOLERANS" oldugu ANDA durdurulur - ama bridge.get_heading() sensor/
    iletisim zincirinden gecikmeli geldigi ve motor coast ile (aktif fren
    olmadan) durdugu icin, robot bu karardan SONRA da birkac derece daha
    fiziksel olarak donmeye devam edebiliyordu (hedefi asma). Robotun
    ANLIK ACISAL HIZI surekli olculur (EMA ile yumusatilarak); tam
    "kalan <= TOLERANS" aninda, o hiza ORANTILI COK KISA (birkac-birkac
    on ms) bir TERS YONLU fren darbesi uygulanip hemen kesilir - bu,
    ataletle devam eden donusu FIZIKSEL olarak dizginler. (Daha once
    denenen "ongorulu/tahmine dayali erken durma" yaklasimi - hedefe
    varmadan tahmini bir sure kadar erken durmak - acik dongu bir tahmine
    dayandigi ve fiziksel testte tutarsiz sonuclar (bazen hala fazla,
    ince ayar yapilinca eksik donme) verdigi icin TERK EDILDI; yerine bu
    olcume dayali, kapali dongu fren darbesi kondu.) Ayrica motor
    durduktan sonraki "settle" bekleme fazinda (magnetometer sakinlesmesi)
    hem esik sikilastirildi (15->8 derece) hem de heading gercekten
    sabitlenince ERKEN CIKIS eklendi - boylece bu fazda hayali/gurultuye
    dayali derece birikimi riski de azaldi.

    NOT: FREN_KATSAYISI ve FREN_MAKS_SURESI ampirik baslangic degerleridir -
    fiziksel testle ince ayar gerekebilir; fonksiyon govdesindeki yorumlara
    bakin.

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
        if _durdurma_istendi_mi(dur_bayragi):
            print("DURDURMA sinyali alindi - donus baslamadan iptal edildi.")
            return 0.0

        kalibrasyon_bekle(bridge, otonom=otonom)

        if _durdurma_istendi_mi(dur_bayragi):
            print("DURDURMA sinyali alindi - donus baslamadan iptal edildi.")
            return 0.0

        # HIZLI MOD: renk sensoru okumasini gecici kapatip Arduino'nun dongu
        # hizini artiriyoruz - bu, heading verisinin cok daha guncel gelmesini
        # saglar, 'Ana donus sonucu'nun hedefi asma miktarini azaltmasi beklenir
        # (asiri deger, buyuk olcude ornekleme gecikmesinden kaynaklaniyordu).
        bridge.request_fast_mode()
        time.sleep(0.1)

        toplam_donus = 0.0
        onceki_heading = bridge.get_heading()
        if onceki_heading is None:
            print("HATA: Heading okunamadi, donus iptal edildi.")
            return 0.0

        donus_yonu_ayarla(yon)
        print(f"  [DEBUG] Yon ayarlandi: {yon}. IN pinleri: "
              f"IN1={GPIO.input(IN1)} IN2={GPIO.input(IN2)} "
              f"IN3={GPIO.input(IN3)} IN4={GPIO.input(IN4)}")

        if hedef_derece > 45:
            # Buyuk acilarda yumusak baslangic (soft-start): motoru aniden
            # HIZ_NORMAL'a sicratmak yerine kademeli hizlandir. Ani akim
            # sicramasi (inrush current), motor surucusunden BNO055'e
            # EMI/gurultu binmesine yol aciyor olabilir - bu, o sicramayi
            # yumusatmayi dener. 45 derece esigi: toplam donus suresi yeterince
            # uzun oldugu icin ~150ms'lik rampa, motorun tork kazanmasini
            # engellemeyecek kadar kisa bir pay kapliyor.
            YUMUSAK_BASLANGIC_ADIMLARI = 5
            for adim in range(1, YUMUSAK_BASLANGIC_ADIMLARI + 1):
                if _durdurma_istendi_mi(dur_bayragi):
                    motorlari_durdur(pwm_a, pwm_b)
                    print("DURDURMA sinyali alindi - soft-start sirasinda iptal edildi.")
                    return toplam_donus
                gecici_hiz = HIZ_NORMAL * adim / YUMUSAK_BASLANGIC_ADIMLARI
                pwm_a.ChangeDutyCycle(gecici_hiz)
                pwm_b.ChangeDutyCycle(gecici_hiz)
                time.sleep(0.03)
            print(f"  [DEBUG] Soft-start tamamlandi, son duty: {gecici_hiz:.1f}")
        else:
            # Kucuk acilarda (<=45 derece) toplam donus suresi zaten kisa
            # (orn. 17.6 derece icin ~180ms) - rampa suresi bu sureyi domine
            # edip motorun statik surtunmeyi yenecek torka hic ulasamamasina
            # (gercek stall) yol acabiliyor. Kucuk acilarda direkt tam hizla basla.
            pwm_a.ChangeDutyCycle(HIZ_NORMAL)
            pwm_b.ChangeDutyCycle(HIZ_NORMAL)
            print(f"  [DEBUG] Direkt tam hiz uygulandi: HIZ_NORMAL={HIZ_NORMAL}")

        yavas_moda_gecildi = False
        cok_yavas_moda_gecildi_donus = False
        # ONCEKI YAKLASIM: yavaslama SADECE hedef_derece > YAVASLAMA_ESIGI (30)
        # oldugunda aktifti - kucuk hedeflerde (<=30 derece) robot hic
        # yavaslamadan tam hizda gidip HEP asiyordu (20 derece hedefte 7-10
        # derece asma gozlemlendi). Bunun sebebi, kucuk hedeflerde 'kalan'
        # DAHA ILK adimda esigin altinda olacagi icin yavaslamanin ANINDA
        # (hic momentum kazanmadan) tetiklenip stall'a yol acmasiydi.
        #
        # YENI YAKLASIM: Yavaslamayi hedef buyuklugune gore degil, bir
        # MINIMUM SURE/momentum kazanma payina gore kapatiyoruz - bu sayede
        # kucuk hedefler de yavaslamadan faydalanabiliyor, ama yine de
        # motor once gercekten HIZ_NORMAL'de bir sure calisip tork/momentum
        # kazanmadan yavaslama tetiklenmiyor.
        MINIMUM_HIZLI_SURE = 0.15  # saniye - bu sure dolmadan yavaslama kontrolu YAPILMAZ
        baslangic_zamani = time.time()
        son_gecerli_veri_zamani = time.time()
        bekleyen_deger_ana = None
        MAKS_GECERLI_ANA_ADIM = 30.0  # derece - 20ms'de bundan buyuk gercek olamaz (motor bu kadar hizli donemez)
        son_debug_zamani = 0.0

        # BNO055 KILITLENME tespiti: veri akisi devam etse bile (is_stale=False),
        # eger heading DEGERI belirli bir sure hic degismezse (motor aktif
        # donerken), bu sensorun kilitlenip ayni degeri tekrarladigi anlamina
        # gelir - bilinen bir BNO055 firmware davranisi. Bu, is_stale()'den
        # FARKLI bir kontrol: is_stale sadece veri akisinin ZAMANLAMASINA bakar,
        # burada ise verinin ICERIGININ degisip degismedigine bakiyoruz.
        KILITLENME_ESIGI = 0.5  # saniye - bu sure boyunca heading hic degismezse kilitlenme say
                                  # (kisa tutuldu: motor tam hizdayken uzun esik = fazla kontrolsuz donus)
        BASLAMA_PAYI = 0.8       # saniye - donus basladiktan sonra bu sure icinde kilitlenme
                                  # KONTROLU YAPILMAZ. Motorun statik surtunmeyi yenip gercekten
                                  # harekete gecmesi bazen ilk birkac yuz ms'yi alabiliyor - bu
                                  # payi tanimadan kontrol edersek, gercekten calisan ama yavas
                                  # baslayan bir donusu yanlislikla "kilitlenme" saniriz.
        DEGISIM_EPSILON = 0.3   # derece - bunun altindaki farklar "degismedi" sayilir
        STALL_KURTARMA_ESIGI = 0.35  # saniye - HIZ_YAVAS'tayken bu sure degismezse once kurtarma dene
                                       # (KILITLENME_ESIGI'den kisa - kurtarma, sert iptalden once denenir)
        son_degisim_zamani = time.time()
        son_bilinen_deger = onceki_heading

        # HIZ ANOMALISI tespiti: "tek teker donuyor" gibi durumlarda heading
        # TAMAMEN donmaz (yukaridaki kilitlenme kontrolu bunu yakalayamaz),
        # ama beklenenden COK daha yavas ilerler (robot pivot yerine kayarak
        # donuyor demektir). Bunu ayri bir kontrolle yakaliyoruz: belirli bir
        # pencerede (PENCERE_SURESI) gerceklesen aciyi, o anki duty icin
        # beklenen minimum aciyla karsilastiriyoruz.
        RATE_KATSAYISI_MIN = 1.2  # derece/saniye, duty basina - HIZ_NORMAL=30'da normal ~90-100 derece/s
                                    # gozlemlendi (~3/duty), guvenlik icin bunun COK altinda bir esik (1.2/duty) kullaniyoruz
        PENCERE_SURESI = 0.4     # saniye - bu sure icinde ne kadar donuldugune bakariz
        pencere_baslangic_zamani = time.time()
        pencere_baslangic_heading = onceki_heading
        mevcut_duty = HIZ_NORMAL

        # =====================================================================
        # AKTIF FREN DARBESI (brake pulse) - OVERSHOOT'UN ANA SEBEBINE COZUM
        #
        # SORUN: bridge.get_heading() ANLIK gercek aciyi degil, I2C okuma ->
        # Arduino dongusu -> seri iletim -> Python thread zincirinde birikmis
        # bir GECIKMEYLE gelen bir onceki ornegi verir. Ayrica motor "coast"
        # ile (aktif fren olmadan) durur, yani sinyal kesildikten sonra da
        # ataletle donmeye devam eder. Bu ikisi birlikte "gereginden fazla
        # donme" (overshoot) sikayetinin ana sebebiydi.
        #
        # ONCEKI DENEME (kaldirildi): "Ongorulu durma" - hedefe varmadan,
        # tahmini bir GECIKME_SURESI kadar ERKEN durarak overshoot'u
        # telafi etmeye calisiyordu. Bu SAF TAHMINE dayaniyordu (acik
        # dongu) - gercek momentum/surtunme/pil gerilimi degiskenligini
        # olcmuyordu, sadece "muhtemelen bu kadar sürer" varsayiyordu. Fiziksel
        # testte tutarsiz sonuclar verdi (bazen hala fazla donuyor, ince
        # ayar yapilinca bu sefer eksik donuyordu) - bu yuzden TAMAMEN
        # KALDIRILDI.
        #
        # YENI YAKLASIM: Robotun O ANKI ACISAL HIZINI (derece/saniye,
        # EMA ile yumusatilmis) olcmeye devam ediyoruz, ama bunu bir "ne
        # zaman durayim" tahmini icin degil, "durma aninda momentumu
        # SONDURMEK icin ne kadar guclu bir fren atisi gerekir" hesabi
        # icin kullaniyoruz. "kalan <= TOLERANS" oldugu GERCEK anda (tahmin
        # degil), motor once durdurulur, SONRA o anki hiza ORANTILI, COK
        # KISA (birkac-birkac on ms) bir TERS YONLU darbe uygulanip hemen
        # kesilir - bu, ataletle devam eden donusu FIZIKSEL olarak
        # dizginler. Daha once denenip "motor kilitlenmesine" yol actigi
        # icin terk edilen SÜREKLİ aktif fren (IN1=IN2=HIGH kisa devre)
        # ile KARISTIRILMASIN - bu COK KISA ve TERS YONLU bir darbe,
        # sürdürülen bir kilitleme degil, bu yuzden ayni riski tasimiyor.
        #
        # FREN_KATSAYISI ve FREN_MAKS_SURESI AMPIRIK BASLANGIC
        # DEGERLERIDIR - fiziksel testle ince ayar gerekebilir:
        #   - HALA fazla donuyorsa (overshoot devam ediyorsa): FREN_KATSAYISI'ni
        #     BUYUT (daha guclu/uzun fren) YA DA FREN_DUTY'yi arttir.
        #   - Robot hedefin GERISINDE kaliyorsa (ters yonde asiriya
        #     kaciyorsa): FREN_KATSAYISI'ni KUCULT.
        # =====================================================================
        FREN_KATSAYISI = 2.1 # saniye / (derece/saniye) - anlik hiza orantili fren suresi
        FREN_MAKS_SURESI = 0.09   # saniye - GUVENLIK TAVANI, bunun uzerine cikmaz
        FREN_MIN_HIZ_ESIGI = 5.0  # derece/saniye - bu hizin altinda fren atisi YAPILMAZ
                                    # (zaten neredeyse duruyorsa ters darbe gereksiz/zararli olur)
        FREN_DUTY = HIZ_YAVAS      # ters yonde uygulanan duty - kontrollu, TAM HIZ DEGIL
        HIZ_EMA_ALPHA = 0.4  # yeni hiz ornegine verilen agirlik (0-1) - dusuk deger daha
                               # yumusak/gecikmeli ama gurultuye dayanikli bir tahmin verir,
                               # yuksek deger daha hizli tepki ama gurultuye daha duyarli olur
        # BUG DUZELTMESI ("103 derece donemedi" sorunu, hala GECERLI): tek bir
        # gurultulu/EMI kaynakli ornek (ozellikle motor daha yeni harekete
        # gecerken, magnetik girisimin en yuksek oldugu anda), EMA'nin ILK
        # agirlikli orneginde fiziksel olarak IMKANSIZ bir hiz uretebiliyordu
        # (orn. 600+ derece/s) - bu da fren darbesini gereksiz yere COK UZUN
        # yapip robotu TERS yonde fazla dondurebilirdi. MAKS_GECERLI_HIZ, tek
        # bir ornegin EMA'ya bu kadar buyuk bir sicramayla giremeyecegi bir
        # tavan koyar.
        MAKS_GECERLI_HIZ = 300.0  # derece/saniye - HIZ_NORMAL=45 duty'de bile normalde
                                    # gozlemlenen ~90-150 derece/s'nin COK uzerinde, guvenli bir tavan
        anlik_hiz_derece_s = 0.0  # EMA ile guncellenen, o anki tahmini acisal hiz
        son_hiz_olcum_zamani = time.time()

        while True:
            if _durdurma_istendi_mi(dur_bayragi):
                print("DURDURMA sinyali alindi - ana donus dongusu aninda iptal ediliyor.")
                break

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

            # [DEBUG] Canli goruntu - motor gercekten donuyor mu, heading
            # gercekten degisiyor mu, Vcc geriliminde dusme var mi (guc
            # yetersizligi teshisi), ikisini ayirt edebilmek icin.
            gecen_sure = time.time() - baslangic_zamani
            if gecen_sure - son_debug_zamani >= 0.1:  # ~her 100ms'de bir
                vcc = bridge.get_vcc()
                vcc_str = f"{vcc:.0f}mV" if vcc is not None else "?"
                print(f"  [DEBUG] t={gecen_sure:.2f}s heading={simdiki_heading} "
                      f"toplam_donus={toplam_donus:.1f} Vcc={vcc_str}")
                son_debug_zamani = gecen_sure

            # KILITLENME kontrolu - veri akiyor ama deger hic degismiyor mu?
            # BASLAMA_PAYI suresi dolmadan bu kontrolu yapma (motor daha
            # statik surtunmeyi yeniyor olabilir).
            if simdiki_heading is not None:
                if son_bilinen_deger is None or abs(aci_farki(son_bilinen_deger, simdiki_heading)) > DEGISIM_EPSILON:
                    son_bilinen_deger = simdiki_heading
                    son_degisim_zamani = time.time()
                elif (yavas_moda_gecildi and gecen_sure > BASLAMA_PAYI and
                      time.time() - son_degisim_zamani > STALL_KURTARMA_ESIGI):
                    # HIZ_YAVAS'ta stall olmus olabilir - sert kilitlenme
                    # tespitine gitmeden once kisa bir kurtarma atisi dene.
                    print(f"  (STALL supheli - HIZ_YAVAS'ta takili kaldi, "
                          f"kurtarma atisi yapiliyor...)")
                    pwm_a.ChangeDutyCycle(HIZ_NORMAL)
                    pwm_b.ChangeDutyCycle(HIZ_NORMAL)
                    time.sleep(0.15)
                    pwm_a.ChangeDutyCycle(HIZ_YAVAS)
                    pwm_b.ChangeDutyCycle(HIZ_YAVAS)
                    son_degisim_zamani = time.time()  # sayaci sifirla
                elif (gecen_sure > BASLAMA_PAYI and
                      time.time() - son_degisim_zamani > KILITLENME_ESIGI):
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

                # ---- Acisal hiz tahminini guncelle (EMA) ----
                simdi_hiz = time.time()
                dt_hiz = simdi_hiz - son_hiz_olcum_zamani
                if dt_hiz > 0:
                    anlik_ornek_hizi = abs(delta) / dt_hiz
                    # BUG DUZELTMESI: tek bir gurultulu ornek fiziksel olarak
                    # imkansiz bir hiz uretmesin diye tavan uygulaniyor -
                    # bkz. MAKS_GECERLI_HIZ tanimindaki not.
                    anlik_ornek_hizi = min(anlik_ornek_hizi, MAKS_GECERLI_HIZ)
                    anlik_hiz_derece_s = (HIZ_EMA_ALPHA * anlik_ornek_hizi +
                                           (1 - HIZ_EMA_ALPHA) * anlik_hiz_derece_s)
                son_hiz_olcum_zamani = simdi_hiz

            kalan = hedef_derece - abs(toplam_donus)

            if kalan <= TOLERANS:
                # ---- AKTIF FREN DARBESI: gercekten hedefe ulasildigi anda,
                # o anki hiza orantili KISA bir ters yonlu darbeyle momentumu
                # sondur. Bu, "coast" ile duran robotun ataletle devam edip
                # hedefi asmasini (overshoot) engelleyen ASIL mekanizma -
                # tahminle degil, olculen gercek hizla orantili. ----
                if anlik_hiz_derece_s > FREN_MIN_HIZ_ESIGI:
                    fren_suresi = min(FREN_MAKS_SURESI, anlik_hiz_derece_s * FREN_KATSAYISI)
                    ters_yon_fren = "sag" if yon == "sol" else "sol"
                    print(f"  [DEBUG] Fren darbesi: kalan={kalan:.1f} derece, "
                          f"anlik_hiz={anlik_hiz_derece_s:.0f} derece/s, "
                          f"fren_suresi={fren_suresi*1000:.0f}ms ({ters_yon_fren} yonde)")
                    donus_yonu_ayarla(ters_yon_fren)
                    pwm_a.ChangeDutyCycle(FREN_DUTY)
                    pwm_b.ChangeDutyCycle(FREN_DUTY)
                    time.sleep(fren_suresi)
                else:
                    print(f"  [DEBUG] Hedefe ulasildi: kalan={kalan:.1f} derece, "
                          f"anlik_hiz={anlik_hiz_derece_s:.0f} derece/s (fren esiginin "
                          f"altinda, darbe uygulanmadi)")
                break

            # HIZ ANOMALISI kontrolu: "tek teker donuyor" gibi durumlarda
            # heading TAMAMEN donmaz ama beklenenden cok yavas ilerler.
            # PENCERE_SURESI dolunca, o sure icinde gercekten kac derece
            # donuldugune bakip, o anki duty icin beklenen minimumla
            # karsilastiriyoruz.
            if gecen_sure > BASLAMA_PAYI and time.time() - pencere_baslangic_zamani >= PENCERE_SURESI:
                if pencere_baslangic_heading is not None and onceki_heading is not None:
                    pencere_aci = abs(aci_farki(pencere_baslangic_heading, onceki_heading))
                    pencere_sure_gercek = time.time() - pencere_baslangic_zamani
                    gozlenen_oran = pencere_aci / pencere_sure_gercek if pencere_sure_gercek > 0 else 0
                    beklenen_min_oran = mevcut_duty * RATE_KATSAYISI_MIN

                    if gozlenen_oran < beklenen_min_oran:
                        print(f"  (HIZ ANOMALISI - beklenen ~{beklenen_min_oran:.0f} derece/s, "
                              f"gozlenen {gozlenen_oran:.0f} derece/s. Muhtemelen tek teker "
                              f"donuyor. Kurtarma denemesi yapiliyor...)")
                        motorlari_durdur(pwm_a, pwm_b)
                        time.sleep(0.15)
                        donus_yonu_ayarla(yon)
                        pwm_a.ChangeDutyCycle(HIZ_NORMAL)
                        pwm_b.ChangeDutyCycle(HIZ_NORMAL)
                        time.sleep(0.15)
                        pwm_a.ChangeDutyCycle(mevcut_duty)
                        pwm_b.ChangeDutyCycle(mevcut_duty)

                pencere_baslangic_zamani = time.time()
                pencere_baslangic_heading = onceki_heading

            # Hedefe yaklasinca yavasla (overshoot'u azaltmak icin) - SADECE
            # hedef acinin gercekten YAVASLAMA_ESIGI'nden buyuk oldugu
            # durumlarda (yani robotun once gercekten HIZ_NORMAL'de bir
            # 'cruise' fazi oldugunda). Kucuk hedeflerde bu kontrolu atlariz.
            #
            # IKI KADEMELI yavaslama: Arduino'nun renk sensoru okumasi ~50-100ms
            # surdugu icin, heading verisi de bu hizda guncelleniyor. HIZ_YAVAS'ta
            # bu gecikme suresince robot birkac derece 'kor' ilerliyor (olcum
            # gecikmesi - motor coast'u degil, once oyle sanmistik). Hedefe iyice
            # yaklasinca DAHA DA yavaslayarak bu 'kor mesafeyi' kucultuyoruz.
            gercek_gecen_sure = time.time() - baslangic_zamani
            yavaslama_izinli = gercek_gecen_sure > MINIMUM_HIZLI_SURE

            if yavaslama_izinli and kalan <= COK_YAVAS_ESIGI_DONUS and not cok_yavas_moda_gecildi_donus:
                pwm_a.ChangeDutyCycle(HIZ_COK_YAVAS_DONUS)
                pwm_b.ChangeDutyCycle(HIZ_COK_YAVAS_DONUS)
                cok_yavas_moda_gecildi_donus = True
                yavas_moda_gecildi = True  # 1. kademeyi de gecmis sayilir
                mevcut_duty = HIZ_COK_YAVAS_DONUS
            elif yavaslama_izinli and kalan <= YAVASLAMA_ESIGI and not yavas_moda_gecildi:
                pwm_a.ChangeDutyCycle(HIZ_YAVAS)
                pwm_b.ChangeDutyCycle(HIZ_YAVAS)
                yavas_moda_gecildi = True
                mevcut_duty = HIZ_YAVAS

            time.sleep(0.02)  # ~50Hz kontrol dongusu

        motorlari_durdur(pwm_a, pwm_b)

        if _durdurma_istendi_mi(dur_bayragi):
            print("DURDURMA sinyali - motor durduruldu, donus fonksiyonundan hemen cikiliyor "
                  "(settle/ince duzeltme adimlari atlaniyor).")
            return toplam_donus

        # Motor calisirken olusan manyetik girisimin sonmesi icin bekle.
        # ONEMLI: Bu sirada motor KAPALI, yani robot fiziksel olarak hareket etmiyor.
        # Bu yuzden bu pencerede gorulen buyuk bir ani sicrama gercek bir donus OLAMAZ -
        # sensorun toparlanma surecindeki gecici bir gurultu/glitch'tir. Tek bir
        # once/sonra olcumu yerine surekli orneklyip, supheli buyuk sicramalari
        # reddederek (ama ayni deger tekrar ederse gercek kabul ederek) daha
        # guvenilir bir sonuc elde ediyoruz.
        SETTLE_SURESI = 1.5
        SETTLE_ORNEK_ARALIGI = 0.1
        # GUNCELLEME (overshoot azaltma): 15.0 -> 8.0. Motor tamamen
        # durduktan sonra GERCEK fiziksel donus cok kucuk olmali (sadece
        # ataletin sonlanmasi kadar) - 15 derecelik bir esik, magnetometer
        # gurultusunden kaynaklanan HAYALI sicramalarin "dogrulanmis" sayilip
        # toplam_donus'a yanlislikla eklenmesine (ve sonrasinda ince_duzeltme_
        # yap()'in bu sisirilmis degere gore yanlis yon/miktar duzeltmesi
        # yapmasina) izin veriyordu. 8 derece, gercek atalet donusunu hala
        # kabul edecek kadar genis ama gurultuyu daha iyi eliyor.
        MAKS_GECERLI_SETTLE_ADIMI = 8.0  # derece - motor kapaliyken bundan buyugu supheli sayilir

        # GUNCELLEME (erken cikis): eskiden settle dongusu HER ZAMAN tam
        # SETTLE_SURESI (1.5s) kadar surerdi - bu sure boyunca gelen HER
        # gurultulu ornek toplam_donus'a (kucuk de olsa) katki yapma riski
        # tasiyordu. Heading belirli bir sure (SETTLE_SESSIZLIK_ESIGI)
        # hic degismezse (gercekten sakinlesmisse), tam 1.5s'yi beklemeden
        # ERKEN CIKILIYOR - hem daha az gurultu birikme riski hem de daha
        # hizli bir donus dongusu.
        SETTLE_SESSIZLIK_ESIGI = 0.5  # saniye - bu sure hic degisim olmazsa erken cik
        SETTLE_DEGISIM_EPSILON = 0.3  # derece - bunun altindaki farklar "degismedi" sayilir

        print(f"Motor durdu, magnetometer'in sakinlesmesi icin en fazla {SETTLE_SURESI}s "
              f"bekleniyor (supheli sicramalar filtreleniyor, {SETTLE_SESSIZLIK_ESIGI}s "
              f"sessizlik olursa erken cikilir)...")

        bekleyen_deger_settle = None
        son_settle_degisim_zamani = time.time()
        son_settle_bilinen_deger = onceki_heading

        settle_baslangic = time.time()
        while time.time() - settle_baslangic < SETTLE_SURESI:
            if _durdurma_istendi_mi(dur_bayragi):
                print("DURDURMA sinyali - settle bekleme dongusu iptal ediliyor.")
                break
            simdiki_heading = bridge.get_heading()
            delta, onceki_heading, bekleyen_deger_settle = guvenli_heading_guncelle(
                onceki_heading, simdiki_heading, bekleyen_deger_settle, MAKS_GECERLI_SETTLE_ADIMI
            )
            if delta is not None:
                toplam_donus += delta

            # Erken cikis kontrolu: heading gercekten sakinlesti mi?
            if simdiki_heading is not None:
                if (son_settle_bilinen_deger is None or
                        abs(aci_farki(son_settle_bilinen_deger, simdiki_heading)) > SETTLE_DEGISIM_EPSILON):
                    son_settle_bilinen_deger = simdiki_heading
                    son_settle_degisim_zamani = time.time()
                elif time.time() - son_settle_degisim_zamani > SETTLE_SESSIZLIK_ESIGI:
                    print(f"  [DEBUG] Settle erken bitti - heading {SETTLE_SESSIZLIK_ESIGI}s'dir "
                          f"sabit ({simdiki_heading}).")
                    break

            time.sleep(SETTLE_ORNEK_ARALIGI)

        if _durdurma_istendi_mi(dur_bayragi):
            return toplam_donus

        # Kalibrasyon durumunu kontrol et - dusukse manyetik girisim teorisini dogrular
        bridge.request_calibration_status()
        time.sleep(0.2)  # cevabin gelmesini bekle (_read_loop icinde yazdirilir)

        print(f"Ana donus sonucu: {toplam_donus:.1f} derece "
              f"(fark: {abs(hedef_derece - abs(toplam_donus)):.1f} derece)")

        # ---- Ince duzeltme: hata FINE_TOLERANS'in altina inene kadar kucuk atislarla duzelt ----
        hedef_isaretli = -hedef_derece if yon == "sol" else hedef_derece
        toplam_donus, onceki_heading = ince_duzeltme_yap(
            bridge, pwm_a, pwm_b, hedef_isaretli, toplam_donus, onceki_heading,
            dur_bayragi=dur_bayragi,
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
