"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

ozel_navigasyon_testi_esc.py'nin ayni iskeletini kullanir (1-4. adimlar
ayni: 30 derece sag -> 0.5s duz -> 60 derece sag -> 0.3s duz), ama HER
atis pozisyonunda kullanicidan yon/aci SORMAK yerine SABIT bir baslangic
acisiyla doner, ESC'yi calistirir, sonra kullanicidan komut gelene kadar
2'şer derecelik adimlarla ACISAL TARAMA (sweep) yaparak ayni bolgeye
farkli acilardan atmayi dener.

GUNCELLEMELER (bu surum):
  1) SKOR DINLEYICI: sweep dongusu artik sadece klavye komutuyla
     ('devam'/'sonraki') degil, break-beam sensorlerinden (Arduino R4
     WiFi -> Flask /skor route -> SkorDinleyici) gelen "top cemberden
     gecti" bilgisiyle de bitebiliyor. GUNCELLEME (kural degisikligi):
     bir pozisyonda ILK basarili gecis sayilinca, o pozisyondan en fazla
     IKINCI_ATIS_MAKS_BEKLEME_SN (2.0s) daha beklenir - bu sure icinde
     2. gecis gelse de gelmese de (yarisma kurali geregi basarili bir
     atistan sonra sadece 1 ek deneme hakki oldugu icin) sweep otomatik
     durup bir sonraki pozisyona geciliyor.
     GUI'deki skor da (bolge_bildir'in dondugu puan uzerinden) her
     gecişte aninda artiyor.
  2) ACIL DURDUR: tum hareket fonksiyonlarina (guvenli_donus,
     ileri_git_sabit_mesafe/sure) bir dur_bayragi (threading.Event)
     gecirilebiliyor. Bu bayrak set edildiginde (Acil Durdur butonu ya
     da Ctrl+C), calisan TUM hareketler en kisa surede durduruluyor ve
     TestDurduruldu istisnasi firlatilarak butun rota GUVENLI sekilde
     ve ANINDA sonlandiriliyor (finally: esc.kapat() her zaman calisir).
  3) SWEEP BEKLEME SURESI: her sweep adiminda beklenen sure 20s'den
     10s'ye dusuruldu.
  4) SWEEP ACISI: 3 dereceden 2 dereceye dusuruldu (POZISYON_1..5
     icindeki sweep_adim degerleri) - daha ince taramali sweep.
  5) 2. POZISYON ACISI: 90 dereceden 93 dereceye guncellendi.
  6) 6. POZISYON GERI EKLENDI (kullanici istegiyle): daha once tamamen
     kaldirilmisti, simdi 7. pozisyonla birlikte GERI eklendi - bkz.
     asagidaki 11. madde.
  7) KIRMIZI->YESIL GECISI (3->4, GECISLER[2]): ultrasonik mesafe
     sensoruyle - onde bir engele (sahanin kenari/duvari) 30cm kalana
     kadar ilerliyor (eskiden 45cm'ydi - robot 45cm'de hala KIRMIZI
     bolgede kaliyordu, yesile hic giremiyordu; esik KUCULTULEREK
     robotun DAHA FAZLA ilerlemesi saglandi). Vardiktan sonra hala renk
     sensoruyle DOGRULANIYOR - yesil algilanmiyorsa kisa ek ileri
     hareketlerle (en fazla 5 kez, eskiden 3'tu) tekrar denenir.

     NOT: Bu deger (esik_cm=30) fiziksel test sonuclarina gore hala
     ayarlanabilir - eger hala kirmizida kaliyorsa GECISLER[2]'deki
     esik_cm degerini daha da kucultun (orn. 20), robot duvara/kenara
     fazla yaklasip carpma riski oluyorsa biraz artirin.
  8) BUYUK ACILI DONUSLER BOLUNUYOR (YENI): 100 dereceden BUYUK her
     donus (pozisyona ilk varista VE baseline'a geri donerken) artik
     TEK PARCA degil, iki esit parcaya bolunerek yapiliyor - ayni
     GECISLER'deki 90 derecelik donusler icin zaten var olan mantik
     (bkz. _donus_uygula), simdi POZISYONLAR'in kendi ilk_aci
     donuslerine de (yeni _buyuk_aci_donus_uygula fonksiyonuyla)
     uygulaniyor. AMAC: buyuk tek parca donuslerde overshoot riskini
     azaltmak - iki kucuk donus, aralarinda ayri ince-duzeltme/settle
     firsati sunuyor, kumulatif hata tek parcaya kiyasla daha dusuk
     kaliyor.
  9) SKOR SAYIMI ARTIK ESC'YE SIKI SIKIYA BAGLI (BUG DUZELTMESI): eskiden
     skor_dinleyici.saymaya_basla() pozisyon_calistir() icinde, ilk donus
     bitince (ESC HENUZ 0'DAYKEN, hatta interaktif ESC GUI'sinde "ESC hizi
     bekleniyor" asamasindayken bile) tetikleniyordu - yani flywheel'lar
     DONMUYORKEN cemberden gecen bir cisim yanlislikla sayilabiliyordu.
     Simdi sayim, ESC gercekten bir hiza ayarlandiktan (esc.hiz_ayarla
     (esc_hiz) cagrisindan) HEMEN SONRA sweep_atis_yap() icinde baslatiliyor,
     ve ESC 0'a dondurulmeden HEMEN ONCE (yine sweep_atis_yap icinde)
     durduruluyor - boylece skor SADECE flywheel'lar fiilen donuyorken
     artabiliyor.
  10) ESC HIZLARI GUNCELLENDI (kullanici istegiyle, SADECE interaktif
     OLMAYAN app.py icin gecerli - interaktif GUI zaten p["esc_hiz"]'i
     YOK SAYIP GUI'den soruyor): kirmizi pozisyonlar (1,2,3) 12.4 -> 12.5,
     yesil pozisyonlar (4,5) 11 olarak SABIT kaldi.
  11) 6. VE 7. POZISYON EKLENDI (kullanici istegiyle):
      - 5->6 gecisi: baseline'a donuldukten sonra 85 derece SOLA donulup
        0.3 saniye ILERI gidiliyor.
      - 6. pozisyon (YESIL, puan=2): 100 derece SAGA donup ESC %11 ile
        atis, sweep SAGA (3 tekrar).
      - 6->7 gecisi (YESIL -> KIRMIZI): baseline'a donuldukten sonra
        (ekstra donus YOK) 0.075 saniye GERIYE gidiliyor - bunun icin
        test_surus.py'ye yeni bir geri_yon_ayarla() fonksiyonu ve
        ileri_git_sabit_sure()'a yon="ileri"/"geri" parametresi eklendi
        (bkz. o dosyalardaki yorumlar). Vardiktan sonra renk sensoruyle
        KIRMIZIYA donuldugu DOGRULANIYOR - hala kirmizi algilanmiyorsa
        kisa ek GERI hareketlerle (en fazla 5 kez) tekrar denenir (bkz.
        _renk_dogrulayarak_ilerle'nin genellenmis hali - artik hem
        hedef_bolge hem yon parametresi alabiliyor, boylece GERIYE
        giden bu geciste ek duzeltme hareketleri de doğru yonde
        (geri) yapiliyor).
      - 7. pozisyon (KIRMIZI, puan=3): 50 derece SAGA donup ESC %12.6
        ile atis, sweep SOLA (3 tekrar - acikca belirtilmedigi icin
        diger pozisyonlarla tutarli sekilde varsayildi). Robot BU
        pozisyondan sonra GERI DONMEZ, rota burada biter.

AMAC: Robotun donuslerde birkac derecelik sapma yasayabilmesi yuzunden
("tam 90 dönemiyor" sorunu) TEK bir sabit aciya guvenmek yerine, o
acinin etrafindaki birkac komsu aciyi da otomatik deneyerek potansiyel
sapmanin sonucunu (top cemberden gecmemesi) telafi etmeye calisir.

POZISYONLAR (7 atis: kirmizidan 4, yesilden 3):
  1. atis (Kirmizi): 87 derece SOL, ESC %12.5, sweep SAGA (2 derece adim), maks 3 tekrar.
  2. atis (Kirmizi): 98 derece SOL, ESC %12.5, sweep SAGA, maks 3.
  3. atis (Kirmizi): 110 derece SOL, ESC %12.5, sweep SAGA, maks 3.
  4. atis (Yesil): 30 derece SOL, ESC %11, sweep SAGA, maks 3.
  5. atis (Yesil): 85 derece SAG, ESC %11, sweep SAGA, maks 3.
  6. atis (Yesil, YENI): 100 derece SAG, ESC %11, sweep SAGA, maks 3.
  7. atis (Kirmizi, YENI): 50 derece SAG, ESC %12.6, sweep SOLA, maks 3.
     Bu pozisyondan sonra robot GERI DONMEZ, kod biter.

GECISLER (pozisyonlar arasi hareket, HER ZAMAN once baseline'a - yani
konuma varilan giris acisina - tam geri donulerek yapilir):
  1->2, 2->3: sadece 10cm ileri (donus yok, ayni kirmizi bolge icinde).
  3->4 (KIRMIZI -> YESIL): 90 derece sola donup 0.075 saniye duz gidilir
       (1.3x hiz carpaniyla), sonra renk sensoruyle yesile girildigi
       dogrulanir.
  4->5: ekstra 90 derece sola donup 10cm ileri gidilir.
  5->6 (YENI): 85 derece sola donup 0.3 saniye ILERI gidilir.
  6->7 (YENI): donus YOK, 0.075 saniye GERIYE gidilir.

SURE BUTCESI (5 dakikalik demo siniri icin kaba tahmin): her sweep
adimi 10s bekleme + donus suresi demek. Tum pozisyonlarda maks 3 tekrar
(~33s'ye kadar her biri). Kullanicinin 'sonraki'/'atla' ile beklemeyi
kisaltmasi ya da 'devam' ile pozisyonu erken bitirmesi bu sureyi daha
da azaltir. Break-beam
sensorlerinden bir basarili gecis gelmesi de (ardindan en fazla 2s
beklenip) bekleme
suresini kisaltir.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(ozel_navigasyon_testi_esc.py, donus_kapali_dongu.py, robot_bridge.py,
test_surus.py, atici_esc_kontrol_pigpio_2.py ve skor_dinleyici.py ile
ayni yerde olmali)
"""

import sys
import os
import select
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
from test_surus import guvenli_donus, ileri_git_sabit_mesafe, ileri_git_engel_bulunca
from atici_esc_kontrol_pigpio_2 import EscKontrol

from ozel_navigasyon_testi_esc import (
    bolge_bildir, bolge_belirle, ileri_git_sabit_sure, sensor_kontrolu,
    MIN_ACIKLIK_CM, ILERLEME_MESAFESI_CM, ESC_PIN,
)


class TestDurduruldu(Exception):
    """
    Acil Durdur (GUI) ya da Ctrl+C ile testin ANINDA ve GUVENLI sekilde
    sonlandirilmasi icin kullanilan sinyal istisnasi. Nerede yakalanirsa
    yakalansin (genelde calistir_ozel_rota_sweep'in en disindaki
    try/except), asagidaki finally: esc.kapat() bloklari HER ZAMAN calisir.
    """
    pass


def _durdurma_kontrol_et(dur_bayragi):
    """dur_bayragi set edilmisse TestDurduruldu firlatir, degilse sessizce doner."""
    if dur_bayragi is not None and dur_bayragi.is_set():
        raise TestDurduruldu()


# ---------- Buyuk acili donusleri bolen ortak esik ----------
BUYUK_ACI_ESIGI = 100.0  # derece - bu esigin USTUNDEKI donusler 2 parcaya bolunur


def _buyuk_aci_donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None):
    """
    Bir donusu uygular. BUYUK_ACI_ESIGI'nin (100 derece) USTUNDEKI aciler
    OTOMATIK olarak iki ESIT parcaya bolunerek yapilir - ayni GECISLER'deki
    90 derecelik donusler icin _donus_uygula()'da kullanilan mantik:
      - Her iki alt-donus de ayri ayri guvenli_donus() cagrisi olarak
        calisir, aralarinda ayri bir ince-duzeltme (fine correction) ve
        settle bekleme adimi olusur - kumulatif hatanin TEK parca buyuk
        bir donuse kiyasla daha dusuk kalmasi beklenir.
      - Ayrica her iki parca da 45 derecenin uzerinde kalabilir (orn. 108
        derece -> 54+54), bu yuzden soft-start rampasi her ikisinde de
        devreye girer - amac rampa atlamak degil, TEK BIR UZUN donus
        yerine iki KISA donus yaparak overshoot birikimini bolmek.

    BUYUK_ACI_ESIGI'nin ALTINDAKI/ESIT aciler (orn. pozisyonlarin kucuk
    ilk_aci degerleri, sweep adimlari) DEGISTIRILMEDEN tek parca donuyor.

    Donus: True (tum alt-donusler basarili) / False (herhangi biri basarisiz)
    """
    if aci > BUYUK_ACI_ESIGI:
        yari = aci / 2.0
        parcalar = [yari, yari]
    else:
        parcalar = [aci]

    for i, parca in enumerate(parcalar, start=1):
        if len(parcalar) > 1:
            olay_fn(f"  {aci} derece {yon} yone donus - "
                    f"parca {i}/{len(parcalar)} ({parca:.1f} derece)...")
        if not guvenli_donus(parca, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            return False
    return True


# ---------- Pozisyon tanimlari (deneysel - kolay ayarlanabilir) ----------
# NOT: sweep_bekleme 20.0 -> 10.0, sweep_adim 3.0 -> 2.0 (kullanici istegiyle).
# 6. VE 7. POZISYONLAR EKLENDI (kullanici istegiyle) - robot artik 7 atis
# pozisyonuna gidiyor (kirmizidan 4, yesilden 3).
#
# GUNCELLEME (BUG DUZELTMESI - "yesil bolgede 3 puan sayiliyordu" sorunu):
# her pozisyona artik SABIT bir "puan" alani eklendi. ONCEDEN puan, o an
# CANLI olarak renk sensorunden okunan bolge_belirle() sonucuna gore
# belirleniyordu - ama robot fiziksel olarak yesil bolgedeyken bile renk
# sensoru bazen hala KIRMIZI okuyabiliyordu (esik degerleri/isik kosullari
# nedeniyle), bu da yesildeki bir atisin YANLISLIKLA 3 puan (kirmizi puani)
# olarak sayilmasina yol aciyordu. Artik puan, HANGI POZISYONDA oldugunuza
# (yani TASARIM GEREGI hangi bolgede olmasi GEREKTIGINE) gore SABIT olarak
# belirleniyor - renk sensoru hala calisiyor (loglarda/dogrulamada
# kullaniliyor - bkz. renk_dogrula, bolge_bildir), ama artik SKORU
# ETKILEMIYOR.
#
# GUNCELLEME (ESC HIZLARI - kullanici istegiyle): kirmizi pozisyonlarin
# (1,2,3) ESC hizi 12.4 -> 12.5. Yesil pozisyonlar (4,5) 11'de SABIT
# kaldi. BU DEGERLER SADECE interaktif OLMAYAN app.py icin GECERLI -
# app_esc_interaktif.py kullaniliyorsa esc_hiz_kontrolcusu devrede oldugu
# icin bu p["esc_hiz"] degerleri YOK SAYILIR, ESC hizi her seferinde
# GUI'den soruluyor (bkz. sweep_atis_yap).
POZISYON_1 = dict(ilk_yon="sol", ilk_aci=87.0, esc_hiz=12.5,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="1. atis", puan=3)
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=98.0, esc_hiz=12.5,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="2. atis", puan=3)
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=110.0, esc_hiz=12.5,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="3. atis", puan=3)
POZISYON_4 = dict(ilk_yon="sol", ilk_aci=30.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="4. atis", puan=2)
POZISYON_5 = dict(ilk_yon="sag", ilk_aci=85.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="5. atis", puan=2)
# GUNCELLEME: 6. ve 7. pozisyonlar eklendi (kullanici istegiyle). 6.
# pozisyon YESIL bolgede (4/5 ile ayni, puan=2). 7. pozisyon KIRMIZI
# bolgede (puan=3) - 6->7 gecisinde (GECISLER[5]) renk_dogrula=True ve
# hedef_bolge="KIRMIZI" ile bu gecis dogrulanip gerekirse ek GERI
# hareketlerle telafi ediliyor.
#
# 7. pozisyonun sweep tekrar sayisi (maks_sweep) acikca belirtilmedigi
# icin diger pozisyonlarla TUTARLI olacak sekilde 3 olarak varsayildi.
#POZISYON_6 = dict(ilk_yon="sag", ilk_aci=100.0, esc_hiz=11.0,
 #                  sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
  #                 maks_sweep=3, etiket="6. atis", puan=2)
#POZISYON_7 = dict(ilk_yon="sag", ilk_aci=50.0, esc_hiz=12.6,
 #                  sweep_yon="sol", sweep_adim=2.0, sweep_bekleme=10.0,
  #                 maks_sweep=3, etiket="7. atis", puan=3)

POZISYONLAR = [POZISYON_1, POZISYON_2, POZISYON_3, POZISYON_4, POZISYON_5]

# ---------- Pozisyonlar arasi gecisler ----------
# GECISLER[i] = POZISYONLAR[i] -> POZISYONLAR[i+1] arasi hareket.
# Her gecis, HER ZAMAN once ilgili pozisyonun baseline'ina (giris acisina)
# tam geri donulduktan SONRA uygulanir (baseline_don() cagrisi ile).
#   ekstra_donus: (yon, derece) ya da None - baseline'a dondukten sonra
#                 EK olarak yapilan bir donus (orn. bolgeler arasi 90 derece)
#   hareket: "mesafe" -> ultrasonik mesafe kontrolu yapip ILERLEME_MESAFESI_CM
#                        kadar ilerler (aciklik yoksa atlar)
#            "sure"   -> sure_s saniye boyunca sabit sureli duz gider
#                        (zamanlamaya dayanir, mesafe olcumu YOK). "yon"
#                        alani ile "ileri" (varsayilan) ya da "geri"
#                        secilebilir (YENI - 6->7 gecisi geriye gidiyor).
#            "engel"  -> esik_cm mesafesindeki bir engele (sahanin kenari/
#                        duvari) kadar ilerler (ileri_git_engel_bulunca ile,
#                        ultrasonik geri bildirimle - "nereye vardigimiz"
#                        kesin olarak bilinir, zamanlamaya guvenilmez)
#   renk_dogrula: True ise, "sure"/"engel" hareketinden SONRA renk sensoruyle
#                 gercekten YESIL bolgeye girilip girilmedigi kontrol
#                 edilir; degilse kisa ek ileri hareketlerle tekrar denenir.
GECISLER = [
    dict(ekstra_donus=None, hareket="mesafe"),                                          # 1 -> 2
    dict(ekstra_donus=None, hareket="mesafe"),                                          # 2 -> 3
    dict(ekstra_donus=("sol", 90.0), hareket="sure", sure_s=0.075, renk_dogrula=True,
         hiz_carpani=1.3 / 1.5),                                                       # 3 -> 4 (kirmizi->yesil, sure ile - 0.075s, 1.3x hiz)
    dict(ekstra_donus=("sol", 90.0), hareket="mesafe"),                                # 4 -> 5
    dict(ekstra_donus=("sol", 85.0), hareket="sure", sure_s=0.3),                      # 5 -> 6 (85 derece sol, 0.3s ileri)
    dict(ekstra_donus=None, hareket="sure", sure_s=0.075, yon="geri",
         renk_dogrula=True, hedef_bolge="KIRMIZI"),                                    # 6 -> 7 (donus YOK, 0.075s GERIYE, kirmiziya donus dogrulanir)
]


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=10.0, skor_dinleyici=None, dur_bayragi=None,
                esc=None, esc_hiz_kontrolcusu=None):
    """
    'sure' saniye boyunca STDIN'den bir komut gelip gelmedigini, TAM
    'sure' kadar bloke olmadan (select() ile non-blocking) kontrol eder.
    Ayrica skor_dinleyici verilmisse, o pozisyonda 2 basarili top gecisi
    olup olmadigini da periyodik kontrol eder. dur_bayragi set edilirse
    TestDurduruldu firlatir (Acil Durdur / Ctrl+C).

    esc + esc_hiz_kontrolcusu (interaktif ESC GUI'si icin) birlikte
    verilirse: her ~0.5s'de bir esc_hiz_kontrolcusu.get() ile GUI'den
    gelen GUNCEL ESC hizi okunur, bir onceki uygulanan degerden farkliysa
    esc.hiz_ayarla() ile ANINDA uygulanir - boylece kullanici sweep
    beklemesi SIRASINDA ESC hizini degistirebilir.

    Uc farkli komut ayirt edilir:
      'devam'     -> kullanici bu POZISYONU tamamen bitirmek istiyor
      'sonraki' (ya da 'atla')
                  -> kullanici sadece bu sweep adimini erken bitirip HEMEN
                     bir sonraki sweep acisina gecmek istiyor - pozisyon
                     bitmiyor, kalan bekleme suresi atlaniyor
      basarili gecis + 2s sinir (skor_dinleyici uzerinden)
                  -> bu pozisyonda break-beam sensorleri topun cemberden
                     BASARIYLA gectigini algiladi. Kural geregi artik
                     sadece 1 EK deneme hakki oldugu icin, ilk basaridan
                     itibaren en fazla 2 saniye daha beklenip (2. gecis
                     gelse de gelmese de) pozisyon OTOMATIK bitirilmeli

    COOLDOWN TELAFISI: skor_dinleyici her tetiklendiginde (basarili
    bir gecis sayildiginda) bir cooldown baslatiyor (bkz. skor_dinleyici.py -
    top potadan sekip geri cikarsa ayni atisin 2 kez sayilmasini engellemek
    icin). Bu cooldown suresinin, sweep'in asil bekleme suresinden (orn.
    10s) CALMAMASI icin: her kontrol turunde skor_dinleyici.
    cooldown_suresini_tuket() cagrilir - yeni baslamis bir cooldown varsa,
    donen sure kadar 'bitis' zaman damgasi ILERI OTELENIR. Yani toplam
    bekleme suresi = sure + (bu pencerede olusan tum cooldown'larin toplami).

    Donus: "devam"     -> kullanici pozisyonu bitirmek istedi
           "sonraki"   -> kullanici bir sonraki sweep acisina hemen gecmek istedi
           "iki_gecis" -> basarili gecis sonrasi 2s sinir doldu (ya da 2. gecis
                        geldi), pozisyon bitirilmeli
           None        -> sure doldu, hicbir sey olmadi (normal akis, sweep'e devam)
    """
    bitis = time.time() + sure
    son_uygulanan_esc_hizi = None
    if esc_hiz_kontrolcusu is not None:
        son_uygulanan_esc_hizi = esc_hiz_kontrolcusu.get()

    while True:
        _durdurma_kontrol_et(dur_bayragi)

        if esc is not None and esc_hiz_kontrolcusu is not None:
            guncel_esc_hizi = esc_hiz_kontrolcusu.get()
            if guncel_esc_hizi != son_uygulanan_esc_hizi:
                esc.hiz_ayarla(guncel_esc_hizi)
                son_uygulanan_esc_hizi = guncel_esc_hizi

        if skor_dinleyici is not None:
            # Yeni baslamis bir cooldown varsa, bekleme suresini o kadar
            # UZAT - boylece cooldown, asil 10s'lik pencereden CALMAZ.
            ek_sure = skor_dinleyici.cooldown_suresini_tuket()
            if ek_sure > 0:
                bitis += ek_sure

        if skor_dinleyici is not None and skor_dinleyici.pozisyon_bitmeli_mi():
            return "iki_gecis"

        kalan = bitis - time.time()
        if kalan <= 0:
            return None

        hazir, _, _ = select.select([sys.stdin], [], [], min(0.5, kalan))
        if hazir:
            girdi = sys.stdin.readline().strip().lower()
            if girdi == "devam":
                return "devam"
            elif girdi in ("sonraki", "atla"):
                return "sonraki"
            elif girdi:
                print(f"  ('{girdi}' anlasilmadi - 'devam' pozisyonu tamamen "
                      f"bitirir, 'sonraki' (ya da 'atla') hemen bir sonraki "
                      f"sweep acisina gecer, bekleme devam ediyor)")


def sweep_atis_yap(bridge, pwm_a, pwm_b, esc, esc_hiz, sweep_yon,
                    sweep_adim, sweep_bekleme, maks_sweep, olay_fn, etiket,
                    skor_dinleyici=None, dur_bayragi=None,
                    esc_hiz_kontrolcusu=None, puan=0):
    """
    ESC'yi esc_hiz'e ayarlayip atisa birakir, sonra su durumlardan biri
    gerceklesene kadar her sweep_bekleme saniyede bir sweep_adim derece
    sweep_yon yonune donup ESC'yi tekrar ayarlar:

      'devam'     -> bu pozisyon tamamen biter (sweep durur, fonksiyon doner)
      'sonraki' (ya da 'atla')
                  -> kalan bekleme suresi atlanip HEMEN bir sonraki sweep
                     acisina gecilir (pozisyon bitmez, sweep sayaci normal
                     isler - sadece bekleme kisaltilmis olur)
      2 basarili gecis (skor_dinleyici uzerinden)
                  -> pozisyon OTOMATIK olarak biter, tipki 'devam' gibi
                     (break-beam sensorleri topun cemberden gectigini
                     algiladi - sweep hemen durur)
      dur_bayragi set edilirse -> TestDurduruldu firlatilir, sweep ANINDA
                  kesilir (ESC durdurma islemi bu fonksiyonun disinda,
                  calistir_ozel_rota_sweep'in finally blogunda yapilir)

    esc_hiz_kontrolcusu (interaktif ESC GUI'si icin) verilirse:
      - esc_hiz parametresi YOK SAYILIR - bunun yerine, pozisyona
        varildiginda GUI'den ilk ESC hizi degeri BEKLENIR (bu bekleme
        sirasinda ESC 0'da/kapali kalir).
      - sweep bekleme dongusu boyunca GUI'den gelen guncellemeler CANLI
        olarak uygulanir (bkz. girdi_bekle).
      None verilirse (varsayilan - interaktif OLMAYAN app.py'de boyle
      cagrilir): esc_hiz parametresi (POZISYONLAR'daki sabit deger)
      otomatik olarak kullanilir.

    ESC KURALI: pozisyon biterken (hangi sebeple olursa olsun - 'devam',
    'sonraki'/zaman asimi yok, 2 gecis, ya da sweep donusu basarisiz) bu
    fonksiyon ESC'yi 0'a (kapali) getirir - robot bir SONRAKI pozisyona
    ULASANA kadar ESC/ucus motorlari BOŞTA kalmali, sadece atis pozisyonunda
    donmeli.

    Hicbir komut/gecis gelmezse (sure dolarsa) normal akista bir sonraki
    sweep adimina geciliyor - yani 'sonraki' ile zaman asiminin sonucu
    AYNI, tek fark 'sonraki' 10s beklemeden hemen tetikliyor olmasi.

    NOT: Konuma varista yapilan ILK donusu CAGIRAN KOD onceden yapmis
    olmali - bu fonksiyon SADECE sweep adimlarini yapar.

    Donus: sweep_toplam (derece) - sweep boyunca sweep_yon yonunde
    TOPLAM ne kadar donuldugu. Geri donerken TERS yonde ayni miktarda
    donmen gerekir.
    """
    if esc_hiz_kontrolcusu is not None:
        ilk_hiz = esc_hiz_kontrolcusu.ilk_deger_bekle(etiket, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)
        if ilk_hiz is None:
            # dur_bayragi set edildigi icin ilk_deger_bekle iptal oldu
            _durdurma_kontrol_et(dur_bayragi)
        esc_hiz = ilk_hiz
        olay_fn(f"  [{etiket}] GUI'den ESC hizi alindi: %{esc_hiz:.1f}, "
                f"atisa birakiliyor...")
    else:
        olay_fn(f"  [{etiket}] ESC hizi %{esc_hiz:.1f} olarak ayarlaniyor, "
                f"atisa birakiliyor...")
    esc.hiz_ayarla(esc_hiz)

    # SKOR SAYIMI: ESC gercekten calismaya BASLADIGI (yani flywheel'lar
    # donmeye basladigi) andan itibaren sayiliyor - daha ONCESI DEGIL.
    if skor_dinleyici is not None:
        skor_dinleyici.saymaya_basla(puan)

    try:
        sweep_toplam = 0.0
        sweep_sayaci = 0
        while True:
            olay_fn(f"  [{etiket}] {sweep_bekleme:.0f}s bekleniyor - basarili bir "
                    f"gecis olursa (kural geregi en fazla 2s daha beklenip) ya da "
                    f"'devam' yazarsan pozisyon biter, 'sonraki' (ya da 'atla') "
                    f"yazarsan beklemeden hemen bir sonraki aciya gecilir...")
            komut = girdi_bekle(sweep_bekleme, skor_dinleyici=skor_dinleyici,
                                 dur_bayragi=dur_bayragi, esc=esc,
                                 esc_hiz_kontrolcusu=esc_hiz_kontrolcusu)

            if komut == "iki_gecis":
                olay_fn(f"  [{etiket}] Basarili atis sonrasi bekleme suresi doldu "
                        f"(ya da 2. basarili gecis geldi) - pozisyon "
                        f"tamamlandi, sweep durduruluyor.")
                break
            if komut == "devam":
                olay_fn(f"  [{etiket}] 'devam' alindi.")
                break

            # komut == "sonraki" veya komut is None (sure doldu) -> her iki
            # durumda da bir sonraki sweep adimina gecilir, tek fark
            # 'sonraki' kalan bekleme suresini atlayip HEMEN gecmesi.
            if komut == "sonraki":
                olay_fn(f"  [{etiket}] 'sonraki' alindi, bekleme atlanip hemen "
                        f"bir sonraki aciya geciliyor...")
            if maks_sweep is not None and sweep_sayaci >= maks_sweep:
                olay_fn(f"  [{etiket}] Maksimum sweep tekrari ({maks_sweep}) doldu.")
                break
            sweep_sayaci += 1
            olay_fn(f"  [{etiket}] Sweep adimi #{sweep_sayaci}: {sweep_adim} "
                    f"derece {sweep_yon}")
            if not guvenli_donus(sweep_adim, sweep_yon, bridge, pwm_a, pwm_b,
                                  dur_bayragi=dur_bayragi):
                _durdurma_kontrol_et(dur_bayragi)
                olay_fn(f"  [{etiket}] UYARI: sweep donusu basarisiz oldu, "
                        f"sweep durduruluyor.")
                break
            sweep_toplam += sweep_adim
            # ESC'yi donus sonrasi tekrar ayarla - bazi ESC/regulator'ler
            # sinyal kesintisinde failsafe'e girip durabiliyor, guvenlik icin.
            # esc_hiz_kontrolcusu varsa GUNCEL degeri kullan (kullanici bu
            # sweep adimi sirasinda hizi degistirmis olabilir).
            guncel_hiz = esc_hiz_kontrolcusu.get() if esc_hiz_kontrolcusu is not None else esc_hiz
            esc.hiz_ayarla(guncel_hiz)

        return sweep_toplam

    finally:
        # ESC KURALI: pozisyon bitti (ne sebeple olursa olsun) - robot bir
        # SONRAKI pozisyona varana kadar ESC BOŞTA kalmali. TestDurduruldu
        # firlasa bile bu finally calisir, ESC guvenli sekilde kapatilir.
        #
        # SIRALAMA ONEMLI: once sayim durduruluyor, SONRA ESC kapatiliyor -
        # boylece ESC kapanma anindaki cok kisa pencerede bile (varsa) gelen
        # bir sensor tetiklemesi yanlislikla sayilmaz.
        if skor_dinleyici is not None:
            skor_dinleyici.saymayi_durdur()
        esc.hiz_ayarla(0)
        olay_fn(f"  [{etiket}] ESC durduruldu (0) - pozisyon tamamlandi.")


def pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn,
                       skor_dinleyici=None, puan=0, dur_bayragi=None,
                       esc_hiz_kontrolcusu=None):
    """
    Bir atis pozisyonunda: ilk donusu yapar (100 dereceden BUYUKSE otomatik
    2 parcaya bolunur - bkz. _buyuk_aci_donus_uygula), ESC'yi ayarlayip
    sweep atisini calistirir. Skor SAYIMI artik BURADA DEGIL, sweep_atis_yap
    icinde ESC gercekten calismaya basladigi anda baslatilip ESC kapanmadan
    hemen once durduruluyor - boylece ilk donus/ESC bekleme gibi flywheel'in
    DONMEDIGI hicbir asamada yanlislikla sayim olmaz.

    esc_hiz_kontrolcusu verilirse: ESC hizi p["esc_hiz"] SABIT degerinden
    DEGIL, GUI'den (interaktif olarak) alinir - bkz. sweep_atis_yap.

    Sweep'i GERI ALMAZ - bu, baseline_don() ile ayri bir adimda yapilir
    (boylece son pozisyonda hic geri donmeden biraktirilabilir).

    Donus: sweep_toplam (derece, sweep_yon yonunde toplam donulen miktar)
    """
    olay_fn(f"\n=== {p['etiket']}: {p['ilk_aci']} derece {p['ilk_yon']} "
            f"yone donuluyor ===")
    if not _buyuk_aci_donus_uygula(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b,
                                     olay_fn, dur_bayragi=dur_bayragi):
        _durdurma_kontrol_et(dur_bayragi)
        raise RuntimeError(f"{p['etiket']}: ilk donus basarisiz oldu")

    return sweep_atis_yap(
        bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
        p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
        olay_fn, p["etiket"], skor_dinleyici=skor_dinleyici,
        dur_bayragi=dur_bayragi, esc_hiz_kontrolcusu=esc_hiz_kontrolcusu,
        puan=puan,
    )


def baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn, dur_bayragi=None):
    """
    Bir pozisyonun sweep'ini geri alip (varsa), sonra ilk donusun
    TERSINE donerek pozisyona GIRIS acisina (baseline, 0 derece relatif)
    tam olarak geri doner. p["ilk_aci"] 100 dereceden BUYUKSE, bu geri
    donus de otomatik 2 parcaya bolunur (bkz. _buyuk_aci_donus_uygula) -
    gidiste bolunuyorsa donuste de ayni sekilde bolunmesi tutarli olur.
    """
    if sweep_toplam > 0:
        olay_fn(f"  {p['etiket']}: sweep geri aliniyor ({sweep_toplam:.1f} "
                f"derece {ters_yon(p['sweep_yon'])})...")
        if not guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b,
                              dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            raise RuntimeError(f"{p['etiket']}: sweep geri alma donusu basarisiz oldu")
    olay_fn(f"  {p['etiket']}: {p['ilk_aci']} derece {ters_yon(p['ilk_yon'])} "
            f"yone donup baslangica (0 derece) geri donuluyor...")
    if not _buyuk_aci_donus_uygula(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b,
                                     olay_fn, dur_bayragi=dur_bayragi):
        _durdurma_kontrol_et(dur_bayragi)
        raise RuntimeError(f"{p['etiket']}: baseline donusu basarisiz oldu")


def _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, maks_ek_adim=5,
                                adim_suresi=0.2, dur_bayragi=None, hiz_carpani=1.0,
                                hedef_bolge="YESIL", yon="ileri"):
    """
    Zamanli gecis hareketinden sonra renk sensoruyle gercekten HEDEF
    bolgeye (varsayilan: YESIL) girilip girilmedigini dogrular. Hala
    hedef bolge algilanmiyorsa, kisa (adim_suresi) ek hareketlerle en
    fazla maks_ek_adim kez dener.

    hedef_bolge: (YENI) dogrulanacak bolgenin adi - "YESIL" ya da
    "KIRMIZI". bolge_belirle()'nin dondugu aciklama metninin bu deger ile
    baslayip baslamadigina bakilir.

    yon: (YENI) ek duzeltme hareketlerinin yonu - "ileri" (varsayilan)
    ya da "geri". Ana gecis hareketi GERIYE yapildiysa (orn. 6->7
    gecisi), ek duzeltme hareketleri de AYNI yonde (geri) yapilmali -
    yoksa robot ana hareketi geri alip yanlis yone ilerlemis olur.

    hiz_carpani: ek hareketler de ANA gecis hareketiyle AYNI hiz
    carpaniyla yapilir (tutarlilik icin).
    """
    for deneme in range(maks_ek_adim + 1):
        _durdurma_kontrol_et(dur_bayragi)

        r, g, b, c = bridge.get_color()
        aciklama, puan = bolge_belirle(r, g, b, c)
        olay_fn(f"  Renk dogrulama: {aciklama} (R={r}, G={g}, B={b})")

        if aciklama.startswith(hedef_bolge):
            olay_fn(f"  Renk dogrulama: {hedef_bolge.lower()} bolgeye ulasildi.")
            return

        if deneme == maks_ek_adim:
            olay_fn(f"  UYARI: Renk dogrulama - maksimum ek adim denendi, hala "
                    f"{hedef_bolge.lower()} algilanmiyor. Devam ediliyor (bolge "
                    f"tespiti atis sirasinda tekrar yapilacak).")
            return

        olay_fn(f"  Renk dogrulama: hala {hedef_bolge.lower()} degil, {adim_suresi}s "
                f"ek {yon} hareket deneniyor ({deneme + 1}/{maks_ek_adim})...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, adim_suresi, dur_bayragi=dur_bayragi,
                              hiz_carpani=hiz_carpani, yon=yon)


def _donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None):
    """
    Bir donusu uygular. 90 DERECELIK donusler OTOMATIK olarak 45+45
    seklinde iki ayri guvenli_donus() cagrisina bolunur - bkz. dosya
    genelinde ayrintili aciklama. 90 disindaki aciler DEGISTIRILMEDEN tek
    parca olarak donuluyor - bu fonksiyon SADECE GECISLER listesindeki
    ekstra_donus (90 derecelik) donusler icin kullaniliyor. (Pozisyonlarin
    kendi buyuk ilk_aci donusleri icin _buyuk_aci_donus_uygula() kullaniliyor.)

    Donus: True (tum alt-donusler basarili) / False (herhangi biri basarisiz)
    """
    if abs(aci - 90.0) < 0.01:
        parcalar = [45.0, 45.0]
    else:
        parcalar = [aci]

    for i, parca in enumerate(parcalar, start=1):
        if len(parcalar) > 1:
            olay_fn(f"  Gecis: {aci} derece {yon} yone donus - "
                    f"parca {i}/{len(parcalar)} ({parca} derece)...")
        else:
            olay_fn(f"  Gecis: {aci} derece {yon} yone (ekstra) donuluyor...")
        if not guvenli_donus(parca, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            return False
    return True


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn, dur_bayragi=None):
    """
    Bir pozisyondan sonrakine gecerken yapilan hareketi uygular
    (baseline_don() cagrildiktan SONRA cagrilmali). GECISLER listesindeki
    bir gecis sozlugunu alir - bkz. dosya basindaki GECISLER aciklamasi.
    """
    if gecis.get("ekstra_donus"):
        yon, aci = gecis["ekstra_donus"]
        if not _donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            raise RuntimeError("Gecis donusu basarisiz oldu")

    _durdurma_kontrol_et(dur_bayragi)

    if gecis["hareket"] == "mesafe":
        mesafe = bridge.get_distance()
        olay_fn(f"  Gecis: ultrasonik mesafe olculdu: {mesafe} cm")
        if mesafe is not None and mesafe > MIN_ACIKLIK_CM:
            olay_fn(f"  Gecis: onde yeterli aciklik var, "
                    f"{ILERLEME_MESAFESI_CM}cm ilerleniyor...")
            ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge,
                                    dur_bayragi=dur_bayragi)
        else:
            olay_fn("  Gecis: onde yeterli aciklik yok, ilerleme atlaniyor.")
    elif gecis["hareket"] == "sure":
        hiz_carpani = gecis.get("hiz_carpani", 1.0)
        yon_sure = gecis.get("yon", "ileri")
        olay_fn(f"  Gecis: {gecis['sure_s']}s duz gidiliyor ({yon_sure}, hiz carpani {hiz_carpani:.3f})...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, gecis["sure_s"], dur_bayragi=dur_bayragi,
                              hiz_carpani=hiz_carpani, yon=yon_sure)

        _durdurma_kontrol_et(dur_bayragi)

        if gecis.get("renk_dogrula"):
            hedef_bolge = gecis.get("hedef_bolge", "YESIL")
            _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi,
                                       hiz_carpani=hiz_carpani, hedef_bolge=hedef_bolge,
                                       yon=yon_sure)
    elif gecis["hareket"] == "engel":
        esik_cm = gecis.get("esik_cm", 45.0)
        esik_telafi_cm = gecis.get("esik_telafi_cm", 25.0)
        efektif_esik = esik_cm + esik_telafi_cm
        hiz_carpani = gecis.get("hiz_carpani", 1.0)

        olay_fn(f"  Gecis: hedef durma mesafesi {esik_cm}cm (telafi payiyla "
                f"sensor esigi {efektif_esik}cm olarak ayarlandi, hiz carpani "
                f"{hiz_carpani:.3f}) - onde engel bulunana kadar ilerleniyor...")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=efektif_esik,
                                           dur_bayragi=dur_bayragi,
                                           hiz_carpani=hiz_carpani)
        _durdurma_kontrol_et(dur_bayragi)
        if not bulundu:
            olay_fn(f"  UYARI: Gecis - {efektif_esik}cm'de engel bulunamadi "
                    f"(zaman asimi), guvenlik amacli durulmustu. Devam ediliyor.")

        if gecis.get("renk_dogrula"):
            hedef_bolge = gecis.get("hedef_bolge", "YESIL")
            _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi,
                                       hiz_carpani=hiz_carpani, hedef_bolge=hedef_bolge)


def calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print, esc_pin=ESC_PIN,
                              skor_dinleyici=None, dur_bayragi=None,
                              esc_hiz_kontrolcusu=None, durum_fn=None):
    """
    calistir_ozel_rota()'nin kullanicidan aci sorma kismini kaldirip
    yerine sabit-aci + otomatik-sweep mantigi koyan, 7 atis pozisyonlu
    (kirmizidan 4, yesilden 3) versiyonu.

    skor_dinleyici verilirse: her pozisyonda break-beam sensorlerinden
    ilk basarili gecisten sonra en fazla 2 saniye daha beklenip (kural
    geregi tek ek deneme hakki taninarak) sweep otomatik durur ve GUI'deki skor,
    pozisyonun SABIT p["puan"] degerine gore (kirmizi=3, yesil=2) aninda
    artar.

    dur_bayragi verilirse (threading.Event): set edildiginde (Acil Durdur
    butonu / Ctrl+C) TUM hareketler en kisa surede durdurulur ve rota
    guvenli sekilde sonlandirilir (esc.kapat() HER ZAMAN calisir).

    esc_hiz_kontrolcusu verilirse (interaktif ESC GUI'si icin): ESC hizi
    POZISYONLAR listesindeki sabit esc_hiz degerlerinden DEGIL, her
    pozisyona varildiginda GUI'den istenip surec boyunca canli
    guncellenebilir sekilde alinir - bkz. sweep_atis_yap. None ise
    (interaktif OLMAYAN app.py'de boyle cagrilir) POZISYONLAR'daki sabit
    esc_hiz degerleri (kirmizi=12.5, yesil=11) otomatik kullanilir.

    ESC KURALI: Robot 1. pozisyona ULASANA kadar (yani asagidaki 1-4.
    baslangic adimlari sirasinda) ESC KAPALI (0) tutulur - ucus motorlari
    sadece bir atis pozisyonunda, sweep_atis_yap tarafindan calistirilir.

    durum_fn (GUI icin YAPILANDIRILMIS pozisyon/bolge bilgisi): verilirse,
    robot her onemli asama gecisinde su sekilde bir sozlukle cagirilir -
    GUI bunu olay_fn'in serbest metin logundan AYRI, ozel bir panelde
    gosterebilir:

        {
            "asama": "atis" | "hareket" | "tamamlandi" | "durduruldu" | "hata",
            "pozisyon_no": int (1-tabanli) | None,
            "toplam_pozisyon": int,
            "etiket": str,           # orn. "3. atis" ya da "3. atis -> 4. atis"
            "bolge": str | None,     # orn. "KIRMIZI (3 puanlik bolge)"
            "puan": int | None,      # o pozisyonun/gecisin bolge puani
        }
    """
    esc = EscKontrol(pin=esc_pin)
    esc.baslat()
    esc.hiz_ayarla(0)  # 1. pozisyona varana kadar ESC KAPALI kalsin

    def _durum_bildir(asama, pozisyon_no=None, etiket="", bolge=None, puan=None):
        if durum_fn is not None:
            durum_fn({
                "asama": asama,
                "pozisyon_no": pozisyon_no,
                "toplam_pozisyon": len(POZISYONLAR),
                "etiket": etiket,
                "bolge": bolge,
                "puan": puan,
            })

    try:
        # ---- 1) Saga 30 derece don ----
        olay_fn("1. adim: 30 derece saga donuluyor...")
        if not guvenli_donus(30, "sag", bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("1. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 2) 0.5 saniye duz git ----
        olay_fn("2. adim: 0.5 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.5, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        # ---- 3) Saga 60 derece don ----
        olay_fn("3. adim: 60 derece saga donuluyor...")
        if not guvenli_donus(60, "sag", bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("3. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 4) 0.3 saniye duz git ----
        olay_fn("4. adim: 0.3 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.3, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        # =====================================================
        # 7 ATIS POZISYONU (kirmizidan 4, yesilden 3)
        # =====================================================
        try:
            for i, p in enumerate(POZISYONLAR):
                # NOT: bolge_bildir() hala cagriliyor - renk sensorunun ne
                # okudugunu LOGLAMAK/GOSTERMEK icin (GUI'deki bolge rozeti
                # icin de kullaniliyor). Ama SKORLAMA icin bu canli okuma
                # DEGIL, pozisyonun kendi SABIT p["puan"] degeri kullaniliyor.
                bolge_aciklama, _canli_puan = bolge_bildir(bridge, olay_fn)
                puan = p["puan"]
                _durum_bildir("atis", pozisyon_no=i + 1, etiket=p["etiket"],
                              bolge=bolge_aciklama, puan=puan)
                sweep_toplam = pozisyon_calistir(
                    bridge, pwm_a, pwm_b, esc, p, olay_fn,
                    skor_dinleyici=skor_dinleyici, puan=puan,
                    dur_bayragi=dur_bayragi,
                    esc_hiz_kontrolcusu=esc_hiz_kontrolcusu,
                )

                son_pozisyon = (i == len(POZISYONLAR) - 1)
                if not son_pozisyon:
                    sonraki_etiket = POZISYONLAR[i + 1]["etiket"]
                    _durum_bildir("hareket", pozisyon_no=i + 1,
                                  etiket=f"{p['etiket']} -> {sonraki_etiket}",
                                  bolge=bolge_aciklama, puan=puan)
                    baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn,
                                 dur_bayragi=dur_bayragi)
                    gecis_uygula(bridge, pwm_a, pwm_b, GECISLER[i], olay_fn,
                                 dur_bayragi=dur_bayragi)
                # son pozisyonda (7. atis) hicbir geri donus/hareket yok
        except RuntimeError as e:
            olay_fn(f"HATA: {e}")
            _durum_bildir("hata", etiket=str(e))
            return False

        olay_fn("\nOzel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(7 atis pozisyonu: kirmizidan 4, yesilden 3).")
        _durum_bildir("tamamlandi", pozisyon_no=len(POZISYONLAR),
                      etiket="Test tamamlandi")
        return True

    except TestDurduruldu:
        motorlari_durdur(pwm_a, pwm_b)
        olay_fn("ACIL DURDUR / iptal sinyali alindi - test guvenli sekilde sonlandirildi.")
        _durum_bildir("durduruldu", etiket="Acil durdur / iptal")
        return False

    finally:
        # BURASI HER ZAMAN CALISIR - normal bitis, hata, RuntimeError ya da
        # TestDurduruldu (Acil Durdur/Ctrl+C) fark etmeksizin ESC kapatilir.
        esc.kapat()


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

    sensor_kontrolu(bridge)
    while True:
        cevap = input("Her sensor duzgun calisiyor, prosedure devam edilsin mi? "
                       "(evet/hayir): ").strip().lower()
        if cevap in ("evet", "e"):
            break
        if cevap in ("hayir", "h"):
            print("Kullanici tarafindan durduruldu.")
            bridge.stop()
            return
        print("Gecerli bir cevap degil - 'evet' ya da 'hayir' yaz.")

    pwm_a, pwm_b = motorlari_ayarla()

    try:
        # NOT: CLI modunda skor_dinleyici/dur_bayragi verilmiyor (Flask/Arduino
        # baglantisi bu context'te yok) - sweep eskisi gibi zaman asimi/klavye
        # komutuyla biter, Ctrl+C ile kesme YALNIZCA normal Python
        # KeyboardInterrupt mekanizmasiyla (asagidaki finally bloguyla) ele
        # alinir. Otomatik 2-gecis durdurmasi ve aninda acil-durdur CLI'da
        # aktif degil - bunlar GUI (app.py) uzerinden calistirinca devreye girer.
        calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print)
    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
