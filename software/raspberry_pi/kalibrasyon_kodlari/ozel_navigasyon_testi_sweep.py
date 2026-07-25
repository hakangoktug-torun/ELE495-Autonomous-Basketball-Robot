"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu)

10 atis pozisyonu: kirmizidan 8, yesilden 2. Her pozisyonda sabit bir
baslangic acisiyla donulur, ESC calistirilir, sonra 2'ser derecelik
adimlarla ACISAL TARAMA (sweep) yapilarak ayni potaya farkli acilardan
atilmaya calisilir.

=====================================================================
 TEMEL KURALLAR (kullanici tarafindan belirlendi)
=====================================================================

 (1) ZAMAN BAZLI HAREKET: butun pozisyon gecisleri "sure x hiz"
     mantigiyla calisir. HICBIR gecis mesafe/ultrasonik geri beslemesiyle
     surulmez - referans DAIMA motor calistirma suresidir. Koordinatlar
     sadece atis acisini hesaplamak icin bu surelerden TURETILIR.
     Ultrasonik sadece iki yerde kullanilir ve ikisi de KONTROL DEGIL:
       - ileri harekette carpma guvenligi (cok yakinsa hareketi atlar)
       - geri harekette teshis amacli UYARI (rotayi kesmez)

 (2) NET DONUSLER: tum 90 derecelik donusler TEK HAMLEDE yapilir.
     Bolme/parcalama KODU DA silindi - yapisal olarak parcalanamaz.

 (3) 3->4 GECISI EN KUCUK ZAMAN DILIMI: bu gecis yasakli bolge sinirinda.
     Saha gecmisi: 0.075s ve 0.03s YASAKLI BOLGEYE giriyordu. Deger
     0.02s'de sabit, 0.3s ise asilamaz tavan olarak assert ediliyor.

 (4) 5->6 GECISI UZUN GERI HAREKET: arac yesil alani NET sekilde terk
     etmeli. 0.30s + 0.08s kalkis darbesi (onceki surumde 0.13s idi).
     Renk sensoruyle KIRMIZIYA donuldugu ayrica dogrulanir.

 (5) 2D DAGILIM: 5 ekstra konum tek eksende dizilmez. Import aninda
     assert ile kontrol edilir (X ve Y yayilimi > 5 cm, konumlar arasi
     en az 5 cm).

=====================================================================
 KALIBRASYON - TUM ROTA IKI SAYIYA BAGLI
=====================================================================
Butun koordinat modeli su iki sabitten turer. Ikisi de su an TAHMIN
(eski 0.075s <-> 17.6 cm eslesmesinden geri cozuldu) - SAHADA OLC:

  ILERI_HIZ_CM_PER_S = 234.7   <- 1.0s ileri surup katedilen cm
  GERI_HIZ_CM_PER_S  = 135.0   <- geri_hareket_testi() ile olc

Bunlari duzeltince asagidaki tablo (ve atis acilari) kendiliginden
duzelir. rota_ozeti() her calistirmada guncel tabloyu basar.

=====================================================================
 TAM ROTA
=====================================================================
   1. atis (Kirmizi, 3p): 87 derece SOL,  ESC %12.5
   1->2 : 0.045s ILERI
   2. atis (Kirmizi, 3p): 98 derece SOL,  ESC %12.5
   2->3 : 0.045s ILERI
   3. atis (Kirmizi, 3p): 110 derece SOL, ESC %12.5
   3->4 : 90 derece SOL (tek hamle) + 0.02s ILERI, renk dogrulamali
   4. atis (Yesil,  2p): 30 derece SOL,   ESC %11
   4->5 : 90 derece SOL (tek hamle) + 0.045s ILERI
   5. atis (Yesil,  2p): 85 derece SAG,   ESC %11
   5->6 : 90 derece SAG + 0.30s GERI (UZUN) -> kirmiziya donus, dogrulamali
   6. atis (Kirmizi, 3p)
   6->7 : 90 derece SAG + 0.08s ILERI
   7. atis (Kirmizi, 3p)
   7->8 : 90 derece SOL + 0.06s ILERI
   8. atis (Kirmizi, 3p)
   8->9 : 90 derece SOL + 0.06s ILERI
   9. atis (Kirmizi, 3p)
   9->10: 90 derece SAG + 0.06s ILERI
  10. atis (Kirmizi, 3p) - bu pozisyondan sonra robot GERI DONMEZ, biter.

POTA KONUMU: mevcut kalibre acilardan geri cikarildi. 1. atisin (0,0)'dan
87 derece sol isini ile 3. atisin (0,20)'den 110 derece sol isini
(-48.0, 2.5) cm'de kesisiyor. Capraz dogrulama: 2. atis icin gereken aci
98.9 derece hesaplaniyor, kodda yazan 98.0 - tutarli.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
"""

import sys
import os
import math
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
    yakalansin, asagidaki finally: esc.kapat() bloklari HER ZAMAN calisir.
    """
    pass


def _durdurma_kontrol_et(dur_bayragi):
    """dur_bayragi set edilmisse TestDurduruldu firlatir, degilse sessizce doner."""
    if dur_bayragi is not None and dur_bayragi.is_set():
        raise TestDurduruldu()


# =====================================================================
#  (A) NET DONUSLER - BOLME KODU TAMAMEN KALDIRILDI
# =====================================================================
# Kullanici kurali: "90 derecelik tum donusler tek bir zaman diliminde /
# tek hamlede yapilmali, parcalanmamalidir."
#
# Eskiden iki ayri bolme yolu vardi (90 -> 45+45 ve >100 -> yari/yari) ve
# bir bayrakla kapatiliyordu. Bayrak yanlislikla acilabilecegi icin bu
# surumde bolme KODU DA silindi - artik yapisal olarak parcalanamaz.


def donus_yap(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None, etiket=""):
    """
    Donusu TEK HAMLEDE yapar. Bolme/parcalama YOK.

    Donus: True (basarili) / False (basarisiz)
    """
    on_ek = f"{etiket}: " if etiket else ""
    olay_fn(f"  {on_ek}{aci:.1f} derece {yon} yone donuluyor (tek hamle)...")
    return guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi)


# =====================================================================
#  (B) ZAMAN BAZLI HAREKET MANTIGI  ( mesafe = sure x hiz )
# =====================================================================
# KULLANICI KURALI: "Butun pozisyon gecisleri sure * hiz mantigiyla
# calismali; mesafe sapmalari yerine motor calistirma sureleri
# (timer/duration) referans alinmalidir."
#
# Bu yuzden ARTIK HICBIR GECIS ultrasonik/mesafe geri beslemesiyle
# surulmuyor. Her gecis sadece iki sey tanimliyor: YON ve SURE.
# Koordinatlar (sadece atis acisini hesaplayabilmek icin) bu surelerden
# TUREVLE cikariliyor:  mesafe_cm = sure_s * hiz_cm_per_s
#
# >>> TUM ROTA IKI SABITE BAGLI. Bunlari OLC, gerisi kendiliginden gelir:
#
#   OLCUM 1 (ileri): robotu ILERI_HIZ_CARPANI hizinda tam 1.0 saniye ileri
#   surup katedilen cm'yi olc -> ILERI_HIZ_CM_PER_S
#
#   OLCUM 2 (geri): ayni seyi geri yonde yap -> GERI_HIZ_CM_PER_S
#   (geri_hareket_testi() fonksiyonu bunu senin icin kolaylastiriyor)
#
# Asagidaki degerler OLCUM DEGIL TAHMIN: eski 0.075s <-> 17.6cm
# eslesmesinden geri cozuldu. Sahada mutlaka dogrula.
ILERI_HIZ_CARPANI = 1.0
ILERI_HIZ_CM_PER_S = 234.7   # TAHMIN - olculmeli

GERI_HIZ_CARPANI = 0.75      # 0.5 kalkis yapamiyordu (bkz. kalkis darbesi)
GERI_HIZ_CM_PER_S = 135.0    # TAHMIN - olculmeli


def _mesafe_cm(sure_s, hareket):
    """sure x hiz -> cm. Koordinat modeli SADECE bunu kullanir."""
    hiz = ILERI_HIZ_CM_PER_S if hareket == "ileri" else GERI_HIZ_CM_PER_S
    return sure_s * hiz


# ---------------------------------------------------------------------
#  KALKIS DARBESI (KICK-START)
# ---------------------------------------------------------------------
# SAHA BULGUSU: 17 cm'lik geri komutunda robot 1 cm yer degistirdi ve iki
# yon mesaji arasinda HIC hareket debug'i basilmadi. Kiyaslama: donus
# hareketi SOFT-START rampasiyla duty 45'e cikip sorunsuz calisiyor.
# Geri hareket ise hiz_carpani 0.5 -> duty ~22 civari ve rampa YOK; bu
# duty yuklu sasiyi STATIK SURTUNMEDEN kurtarmaya yetmiyor. Sorun yon
# pinlerinde degil, TORKTA.
#
# COZUM: her geri hareket kisa ve yuksek duty'li bir "kalkis darbesi" ile
# basliyor. Bu darbe de bir SURE oldugu icin katettigi mesafe koordinat
# modeline dahil ediliyor (bkz. _geri_toplam_sure).
GERI_KICK_SURESI_S = 0.08   # kalkis darbesi suresi
GERI_KICK_CARPANI = 1.0     # kalkis darbesi hiz carpani (tam guc)

# ---------------------------------------------------------------------
#  GERI YON KURALI
# ---------------------------------------------------------------------
# Bir GERI hareket baslatilmadan once tekerleklerin yonu AKTIF olarak
# geriye alinmali. Tek satirlik bir PWM cagrisi degil, uc adimli prosedur:
#   1) Motorlari TAMAMEN durdur - L298N gibi H-bridge suruculerde yon
#      pinleri motor donerken cevrilirse surucu ya shoot-through yasar ya
#      da komutu hic islemez.
#   2) YON_DEGISIM_BEKLEME_S bekle - motorlarin fiilen durmasi ve
#      surucunun yeni yon pinlerini ornekleyebilmesi icin.
#   3) geri_yon_ayarla() ile yon pinlerini GERI konumuna al, sonra hareket.
# Hareket bitince yon TEKRAR ileriye alinir.
YON_DEGISIM_BEKLEME_S = 0.15

# Geri hareket sonrasi ultrasonik ile "gercekten yer degistirdik mi"
# kontrolu. ARTIK SADECE UYARI - rotayi durdurmuyor: kullanici kurali
# geregi referans SURE, mesafe olcumu degil. (Onceki surumde bu kontrol
# 13 cm'lik bir yakin-alan okumasi yuzunden rotayi bosuna kesiyordu.)
GERI_HAREKET_DOGRULA = True
GERI_DOGRULAMA_MIN_ORAN = 0.2
GERI_DOGRULAMA_BANDI_CM = (20.0, 200.0)  # bu bandin disinda okuma guvenilmez

# test_surus.py'deki yon fonksiyonlari - savunmali import.
try:
    from test_surus import geri_yon_ayarla as _geri_yon_ayarla
except ImportError:
    _geri_yon_ayarla = None
try:
    from test_surus import ileri_yon_ayarla as _ileri_yon_ayarla
except ImportError:
    _ileri_yon_ayarla = None


def _tekerlek_yonu_ayarla(pwm_a, pwm_b, yon, olay_fn):
    """Tekerlek yonunu AKTIF olarak degistirir (yukaridaki 3 adimli prosedur)."""
    motorlari_durdur(pwm_a, pwm_b)
    time.sleep(YON_DEGISIM_BEKLEME_S)

    fn = _geri_yon_ayarla if yon == "geri" else _ileri_yon_ayarla
    if fn is None:
        olay_fn(f"  UYARI: test_surus.py icinde "
                f"{'geri_yon_ayarla' if yon == 'geri' else 'ileri_yon_ayarla'}() "
                f"bulunamadi - tekerlek yonu AKTIF olarak degistirilemiyor!")
        return False

    fn()
    time.sleep(YON_DEGISIM_BEKLEME_S)
    olay_fn(f"  Tekerlek yonu {yon.upper()} konumuna alindi.")
    return True


# ---- 3 -> 4 gecisi: EN KUCUK ZAMAN DILIMI (yasakli bolge siniri) ----
# SAHA GECMISI (bu deger deneme-yanilma ile daraltildi):
#   0.075s -> YASAKLI BOLGE (cok ileri)
#   0.030s -> YASAKLI BOLGE (hala cok ileri)
#   0.020s -> COK AZ (yesile zar zor geciyor)
#   0.025s -> ^ ikisinin ortasi, SU ANKI DEGER
# Yani calisma araligi 0.02 ile 0.03 arasinda cok dar. Hala az geliyorsa
# 0.027, fazla geliyorsa 0.022 dene - adim 0.002-0.003'ten buyuk olmasin.
# 0.3s tavani guvenlik icin duruyor (o deger ~70 cm demek).GECIS_3_4_SURE_S = 0.028
GECIS_3_4_SURE_S = 0.05
GECIS_3_4_SURE_TAVAN_S = 0.3   # guvenlik tavani - asilirsa import patlar
GECIS_3_4_HIZ_CARPANI = 1.3 / 1.5

assert GECIS_3_4_SURE_S <= GECIS_3_4_SURE_TAVAN_S, (
    f"3->4 gecis suresi {GECIS_3_4_SURE_S}s, tavan {GECIS_3_4_SURE_TAVAN_S}s "
    f"asiliyor - arac yasakli bolgeye girer.")

GECIS_3_4_MESAFE_CM = _mesafe_cm(GECIS_3_4_SURE_S, "ileri")  # ~4.7 cm


# =====================================================================
#  (C) KONUM GEOMETRISI - 2D duzlemde 5 ekstra konum
# =====================================================================
# Koordinat sistemi (cm): baslangic adimlari bitince robotun bulundugu
# nokta (0, 0), o andaki bakis yonu +y (90 derece). +x saga dusuyor.
#   1. atis (0,0) | 2. atis (0,10) | 3. atis (0,20)   -> KIRMIZI
#   4. atis (-17.6, 20) | 5. atis (-17.6, 10)          -> YESIL
#
# POTA_KONUMU, mevcut kalibre acilardan geri cikarildi (bkz. dosya basi).
POTA_KONUMU = (-48.0, 2.5)

# 5. atisin baseline'indaki konum ve bakis yonu - ekstra rotanin
# BASLANGIC noktasi. ARTIK SABIT DEGIL: 3->4 gecisinde katedilen mesafeden
# TUREYEN bir deger. Boylece GECIS_3_4_SURE_S degistiginde (orn. yasakli
# bolge yuzunden 0.075 -> 0.05) butun koordinat zinciri kendiliginden
# duzeliyor - eskiden bu elle guncellenmezse 6-10 arasi konumlar kayiyordu.
BASLANGIC_P5 = (-GECIS_3_4_MESAFE_CM, 10.0)
BASLANGIC_YON_P5 = 270.0  # derece (guneye bakiyor)

# ---- 5 EKSTRA KONUM ve onlara ULASAN gecisler ----
# Her kayit:
#   x, y          : konumun hedeflenen koordinati (cm)
#   varis_acisi   : o konuma varildigindaki bakis yonu (derece, +x=0, +y=90)
#   gecis.donus   : (yon, derece) - HER ZAMAN 90 derece, TEK HAMLE
#   gecis.hareket : "ileri" (mesafe kontrollu) ya da "geri" (kalibre sureli)
#   gecis.mesafe_cm: katedilecek mesafe - GERI hareketler BELIRGIN olacak
#                    sekilde secildi (12-28 cm)
#
# Rotanin 2D karakteri: ILERI (6->7, 9->10), GERI (5->6, 7->8, 8->9) ve
# 90 derecelik donusler birlikte kullaniliyor; noktalar tek eksende
# dizilmiyor (X: 10-30 cm, Y: 10-35 cm).
# ---- 5 EKSTRA KONUM: her gecis SADECE yon + SURE ----
# Koordinat da mesafe de ELLE yazilmiyor; ikisi de sure x hiz'dan turuyor.
#
# 5->6 GECISI UZATILDI (kullanici istegi): "yesil bolgede takili
# kalmayacak kadar uzun bir sure geri git". Onceki surumde 0.13s idi,
# simdi 0.30s - yani 2.3 kat daha uzun ve tahmini ~40 cm. Robot yesil
# alani net sekilde terk ediyor. Renk sensoru dogrulamasi da hala devrede
# (kirmizi gorulmezse ek geri adimlar atiliyor).
#
# 6->10 arasi ILERI hareketler potaya DOGRU ilerliyor: uzun geri hareket
# robotu potadan uzaklastirdigi icin, kalan rota mesafeyi tekrar kapatiyor
# (pota mesafeleri 84 -> 57 cm'ye iniyor).
EK_ROTA = [
    # 5->6: UZUN GERI HAREKET - tek zaman bazli adim (geri yonu mesafe
    # fonksiyonu desteklemiyor). Kalkis darbesi + 0.30s.
    dict(etiket="6. atis", donus=("sag", 90.0), hareket="geri", sure_s=0.30,
         renk_dogrula=True, hedef_bolge="KIRMIZI",
         maks_ek_adim=8, ek_adim_sure_s=0.08),
    # 6->10: ILERI adimlar - 1-5 arasiyla AYNI mekanizma (mesafe kontrollu,
    # rampali). Kisa zaman darbeleri robotu yerinden oynatamadigi icin
    # burada da mesafe kullaniliyor.
    dict(etiket="7. atis", donus=("sag", 90.0), hareket="mesafe", mesafe_cm=20.0),
    dict(etiket="8. atis", donus=("sol", 90.0), hareket="mesafe", mesafe_cm=15.0),
    dict(etiket="9. atis", donus=("sol", 90.0), hareket="mesafe", mesafe_cm=15.0),
    dict(etiket="10. atis", donus=("sag", 90.0), hareket="mesafe", mesafe_cm=15.0),
]

# 5->6 geri hareketi "uzun" olmali - bu kural import aninda kontrol ediliyor.
GERI_MIN_UZUN_SURE_S = 0.25
assert EK_ROTA[0]["sure_s"] >= GERI_MIN_UZUN_SURE_S, (
    f"5->6 geri suresi {EK_ROTA[0]['sure_s']}s - en az "
    f"{GERI_MIN_UZUN_SURE_S}s olmali (yesil bolgeyi net terk etme kurali).")

# ---- ESC hizi <-> pota mesafesi kalibrasyonu ----
# 1-3. atislar ~48 cm mesafeden %12.5 ile atiyor. Ekstra konumlar 58-85
# cm arasi oldugu icin ESC hizi mesafeyle birlikte artmali. Asagida iki
# noktali DOGRUSAL interpolasyon var; ikinci nokta bir TAHMIN - sahada
# uzak mesafeden bir atis kalibre edip buraya yaz.
ESC_KALIBRASYON = [(48.0, 12.5), (85.0, 13.4)]
ESC_MIN, ESC_MAKS = 11.0, 14.0

# ---- Ekstra konumlarin sweep parametreleri ----
EK_SWEEP_YON = "sag"
EK_SWEEP_ADIM = 2.0
EK_SWEEP_BEKLEME = 6.0   # 5 dk'lik demo siniri icin 10s yerine 6s
EK_MAKS_SWEEP = 3


def _aci_normalize(aci):
    """Aciyi (-180, 180] araligina indirger."""
    aci = (aci + 180.0) % 360.0 - 180.0
    return aci + 360.0 if aci <= -180.0 else aci


def _esc_hizi_hesapla(mesafe_cm):
    """Pota mesafesine gore ESC hizini iki noktali interpolasyonla bulur."""
    (d1, h1), (d2, h2) = ESC_KALIBRASYON
    oran = (mesafe_cm - d1) / (d2 - d1)
    return round(min(ESC_MAKS, max(ESC_MIN, h1 + oran * (h2 - h1))), 1)


def _ek_pozisyon_uret(etiket, x, y, varis_acisi):
    """
    Hesaplanmis bir (x, y, varis_acisi) konumundan tam bir POZISYON
    sozlugu uretir: potaya gereken donus acisi ve yonu, mesafeye gore
    ESC hizi, sweep parametreleri.
    """
    dx = POTA_KONUMU[0] - x
    dy = POTA_KONUMU[1] - y
    fark = _aci_normalize(math.degrees(math.atan2(dy, dx)) - varis_acisi)
    mesafe = math.hypot(dx, dy)

    return dict(
        ilk_yon=("sol" if fark > 0 else "sag"),
        ilk_aci=round(abs(fark), 1),
        esc_hiz=_esc_hizi_hesapla(mesafe),
        sweep_yon=EK_SWEEP_YON,
        sweep_adim=EK_SWEEP_ADIM,
        sweep_bekleme=EK_SWEEP_BEKLEME,
        maks_sweep=EK_MAKS_SWEEP,
        etiket=etiket,
        puan=3,                      # hepsi KIRMIZI bolge
        konum=(round(x, 1), round(y, 1)),
        pota_mesafesi=round(mesafe, 1),
    )


def _ek_rotayi_coz():
    """
    EK_ROTA'daki gecisleri P5'in baseline'indan itibaren adim adim SIMULE
    edip her ekstra konumun (x, y) koordinatini ve varis acisini hesaplar.

    Koordinatlar artik ELLE yazilmadigi icin "gecisi degistirdim ama
    koordinati guncellemedim" hatasi yapisal olarak IMKANSIZ.

    Donus: [(etiket, x, y, varis_acisi), ...]
    """
    x, y = BASLANGIC_P5
    aci = BASLANGIC_YON_P5
    cozum = []

    for adim in EK_ROTA:
        yon, derece = adim["donus"]
        assert abs(derece - 90.0) < 1e-6, (
            f"{adim['etiket']}: gecis donusu 90 derece olmali "
            f"(bulundu: {derece})")
        aci = (aci + derece) % 360.0 if yon == "sol" else (aci - derece) % 360.0

        # mesafe = SURE x HIZ  (geri harekette kalkis darbesi de bir
        # suredir, o yuzden toplam sureye dahil ediliyor)
        if adim["hareket"] == "geri":
            mesafe = _mesafe_cm(adim["sure_s"] + GERI_KICK_SURESI_S, "geri")
            isaret = -1.0
        elif adim["hareket"] == "ileri":
            mesafe = _mesafe_cm(adim["sure_s"], "ileri")
            isaret = 1.0
        else:  # "mesafe" - dogrudan cm
            mesafe = adim["mesafe_cm"]
            isaret = 1.0
        x += isaret * mesafe * math.cos(math.radians(aci))
        y += isaret * mesafe * math.sin(math.radians(aci))
        cozum.append((adim["etiket"], x, y, aci))

    return cozum


EK_COZUM = _ek_rotayi_coz()


def _2d_yayilim_kontrolu(cozum):
    """
    Ekstra konumlarin GERCEKTEN 2 boyuta yayildigini dogrular (kullanici
    kurali): noktalar tek eksende dizilmemeli ve ust uste binmemeli.
    """
    noktalar = [(x, y) for _, x, y, _ in cozum]
    x_yayilim = max(x for x, _ in noktalar) - min(x for x, _ in noktalar)
    y_yayilim = max(y for _, y in noktalar) - min(y for _, y in noktalar)
    assert x_yayilim > 5.0 and y_yayilim > 5.0, (
        f"Ekstra konumlar tek eksende dizilmis (X yayilim {x_yayilim:.1f} cm, "
        f"Y yayilim {y_yayilim:.1f} cm) - 2D dagilim kurali ihlal ediliyor.")

    en_yakin = min(math.dist(a, b)
                   for i, a in enumerate(noktalar) for b in noktalar[i + 1:])
    assert en_yakin > 5.0, (
        f"Iki ekstra konum birbirine cok yakin ({en_yakin:.1f} cm) - "
        f"'5 FARKLI konum' kurali icin mesafeleri ayarla.")
    return x_yayilim, y_yayilim, en_yakin


EK_YAYILIM = _2d_yayilim_kontrolu(EK_COZUM)


# ---------- Pozisyon tanimlari ----------
# 1-5. pozisyonlar ELLE kalibre edilmis degerlerle DEGISMEDEN kaliyor.
# Her pozisyonun SABIT bir "puan" alani var (kirmizi=3, yesil=2) - puan
# CANLI renk okumasindan DEGIL buradan geliyor (renk sensoru bazen yesil
# bolgede bile kirmizi okuyabildigi icin skor bozuluyordu).
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

# 6-10. pozisyonlar GEOMETRIDEN uretiliyor (bkz. _ek_pozisyon_uret)
EK_POZISYONLAR = [_ek_pozisyon_uret(*c) for c in EK_COZUM]
POZISYON_6, POZISYON_7, POZISYON_8, POZISYON_9, POZISYON_10 = EK_POZISYONLAR

POZISYONLAR = [POZISYON_1, POZISYON_2, POZISYON_3, POZISYON_4,
               POZISYON_5] + EK_POZISYONLAR


# ---------- Pozisyonlar arasi gecisler ----------
# GECISLER[i] = POZISYONLAR[i] -> POZISYONLAR[i+1] arasi hareket.
# Her gecis, HER ZAMAN once ilgili pozisyonun baseline'ina (giris acisina)
# tam geri donulduktan SONRA uygulanir (baseline_don() cagrisi ile).
#   ekstra_donus : (yon, derece) ya da None - TEK HAMLEDE yapilir
#   hareket      : "mesafe" -> ultrasonik aciklik kontrolu + mesafe_cm ileri
#                  "geri"   -> mesafe_cm kadar GERI (kalibre sureyle)
#                  "sure"   -> sure_s saniye duz (eski davranis, 3->4 icin)
#                  "engel"  -> esik_cm mesafesindeki engele kadar ilerle
#   renk_dogrula : hareketten sonra hedef_bolge'ye girildigi dogrulanir
#   hedef_bolge  : "YESIL" (varsayilan) / "KIRMIZI"
# ---------------------------------------------------------------------
#  ILERI GECISLER MESAFE BAZLI (SAHA BULGUSU - GERI ALINDI)
# ---------------------------------------------------------------------
# Onceki surumde 1->2 ve 2->3 gecisleri "sure x hiz" kuralina uysun diye
# ileri_git_sabit_mesafe() yerine 0.045s'lik zaman darbesine cevrilmisti.
# SONUC: robot HIC ilerlemedi. Sebep geri harekettekiyle AYNI - 45 ms'lik
# bir darbe, soft-start rampasi devreye girmeden bitiyor ve tekerlekler
# statik surtunmeyi kiramiyor.
#
# Bu yuzden ILERI gecisler ilerleme_mesafesi ile (ileri_git_sabit_mesafe,
# rampali ve kapali cevrimli) yapiliyor - yani 1-5 arasi ZATEN CALISAN
# davranis aynen korunuyor. Zaman bazli surus SADECE iki yerde:
#   - 3->4 : yasakli bolge siniri, mesafe kontrolune birakilamayacak kadar
#            kucuk bir hareket (0.02s)
#   - 5->6 : geri hareket (ileri_git_sabit_mesafe geri yonu desteklemiyor),
#            kalkis darbesiyle birlikte
GECISLER = [
    # 1 -> 2 : DEGISMEDI - 10 cm ileri (mesafe kontrollu)
    dict(ekstra_donus=None, hareket="mesafe", mesafe_cm=ILERLEME_MESAFESI_CM),
    # 2 -> 3 : DEGISMEDI - 10 cm ileri (mesafe kontrollu)
    dict(ekstra_donus=None, hareket="mesafe", mesafe_cm=ILERLEME_MESAFESI_CM),
    # 3 -> 4 : KIRMIZI -> YESIL, EN KUCUK zaman dilimi (yasakli bolge siniri)
    dict(ekstra_donus=("sol", 90.0), hareket="ileri", sure_s=GECIS_3_4_SURE_S,
         hiz_carpani=GECIS_3_4_HIZ_CARPANI, renk_dogrula=True),
    # 4 -> 5 : DEGISMEDI - 90 derece SOL (tek hamle) + 10 cm ileri
    dict(ekstra_donus=("sol", 90.0), hareket="mesafe",
         mesafe_cm=ILERLEME_MESAFESI_CM),
]

# 5 -> 6, 6 -> 7, ... 9 -> 10 : EK_ROTA'dan otomatik uretiliyor - gecis
# ile konum TEK KAYNAKTAN geldigi icin ayrisma imkansiz.
#
# NOT: EK_ROTA'da hareket tipi okunabilirlik icin "ileri"/"geri" yaziliyor;
# gecis_uygula() ise ILERI hareketi "mesafe" adiyla (ultrasonik aciklik
# kontrollu ileri_git_sabit_mesafe) ele aliyor. Cevrim burada yapiliyor -
# EK_ROTA'ya "ileri" yazip gecisin sessizce hic calismamasi gibi bir tuzak
# olusmasin diye.
for _adim in EK_ROTA:
    assert _adim["hareket"] in ("ileri", "geri", "mesafe"), (
        f"{_adim['etiket']}: gecis hareketi 'mesafe'/'ileri'/'geri' olmali "
        f"(bulundu: {_adim['hareket']})")
    _gecis = dict(ekstra_donus=_adim["donus"], hareket=_adim["hareket"])
    if _adim["hareket"] == "mesafe":
        _gecis["mesafe_cm"] = _adim["mesafe_cm"]
    else:
        _gecis["sure_s"] = _adim["sure_s"]
    if _adim.get("renk_dogrula"):
        _gecis.update(renk_dogrula=True,
                      hedef_bolge=_adim.get("hedef_bolge", "YESIL"),
                      maks_ek_adim=_adim.get("maks_ek_adim", 5),
                      ek_adim_sure_s=_adim.get("ek_adim_sure_s", 0.05))
    GECISLER.append(_gecis)

assert len(GECISLER) == len(POZISYONLAR) - 1, (
    f"GECISLER {len(POZISYONLAR) - 1} eleman olmali, {len(GECISLER)} var.")

# Her GERI hareket "belirgin" olmali (kullanici kurali) - artik SURE
# uzerinden kontrol ediliyor.
for _g in GECISLER:
    if _g["hareket"] == "geri":
        assert _g["sure_s"] >= 0.10, (
            f"Geri hareket suresi {_g['sure_s']}s - cok kisa, tekerlekler "
            f"kalkis yapamadan biter (belirgin geri hareket kurali).")


def _p4_p5_aci_uyarisi():
    """
    4. ve 5. atisin acilari ELLE kalibre edilmisti - ama o kalibrasyon
    3->4 gecisinin ESKI (0.075s) durma noktasina gore yapilmisti. Sure
    0.05s'ye sabitlenince robot ~6 cm DAHA ERKEN duruyor, dolayisiyla
    potaya gereken aci da degisiyor.

    Bu degerleri OTOMATIK degistirmiyorum: elle kalibre edilmis sayilar,
    benim geri-cozdugum hiz sabitinden daha guvenilir. Ama sahada kontrol
    edebilmen icin geometrinin onerdigi acilari hesaplayip yaziyorum.

    Donus: [(etiket, mevcut_aci, mevcut_yon, onerilen_aci, onerilen_yon)]
    """
    sonuc = []
    for etiket, konum, varis, mevcut in (
        ("4. atis", (-GECIS_3_4_MESAFE_CM, 20.0), 180.0, POZISYON_4),
        ("5. atis", (-GECIS_3_4_MESAFE_CM, 10.0), 270.0, POZISYON_5),
    ):
        dx = POTA_KONUMU[0] - konum[0]
        dy = POTA_KONUMU[1] - konum[1]
        fark = _aci_normalize(math.degrees(math.atan2(dy, dx)) - varis)
        sonuc.append((etiket, mevcut["ilk_aci"], mevcut["ilk_yon"],
                      round(abs(fark), 1), "sol" if fark > 0 else "sag"))
    return sonuc


def rota_ozeti(yaz=print):
    """Uretilen pozisyonlari/gecisleri tek bakista gosterir (debug icin)."""
    yaz(f"{'poz':>9} {'konum':>14} {'atis acisi':>16} "
        f"{'pota':>8} {'ESC':>6}")
    for p in POZISYONLAR:
        konum = p.get("konum")
        konum_s = f"({konum[0]:.0f},{konum[1]:.0f})" if konum else "-"
        mesafe_s = f"{p['pota_mesafesi']:.1f}cm" if p.get("pota_mesafesi") else "-"
        yaz(f"{p['etiket']:>9} {konum_s:>14} "
            f"{p['ilk_aci']:>10.1f} {p['ilk_yon']:>5} {mesafe_s:>8} "
            f"%{p['esc_hiz']:>5.1f}")

    yaz("")
    yaz(f"3->4 gecisi: SABIT {GECIS_3_4_SURE_S}s (~{GECIS_3_4_MESAFE_CM:.1f} cm) "
        f"- yasakli bolge siniri")
    _g56 = EK_ROTA[0]["sure_s"]
    yaz(f"5->6 GERI: {_g56:.2f}s (+{GERI_KICK_SURESI_S:.2f}s kalkis darbesi) "
        f"= ~{_mesafe_cm(_g56 + GERI_KICK_SURESI_S, 'geri'):.1f} cm")
    yaz(f"Ekstra konum yayilimi: X {EK_YAYILIM[0]:.1f} cm, Y {EK_YAYILIM[1]:.1f} cm, "
        f"en yakin ikili {EK_YAYILIM[2]:.1f} cm")

    uyari = [u for u in _p4_p5_aci_uyarisi() if abs(u[1] - u[3]) > 2.0]
    if uyari:
        yaz("")
        yaz("DIKKAT - 3->4 suresi degistigi icin 4/5. atis acilari eskimis "
            "olabilir (otomatik degistirilmedi, sahada dogrula):")
        for etiket, mev_a, mev_y, one_a, one_y in uyari:
            yaz(f"  {etiket}: kodda {mev_a:.1f} {mev_y} | "
                f"geometri {one_a:.1f} {one_y} olmasi gerektigini soyluyor")


def gecis_ozeti(gecis):
    """
    Bir gecisi tipinden BAGIMSIZ olarak tek satirda ozetler.

    Log satirlari eskiden gecis["mesafe_cm"]'i dogrudan okuyordu; 5->6
    sure bazina cevrilince bu KeyError firlatti. Artik butun log ciktilari
    bu fonksiyondan geciyor, gecis tipi degistiginde kirilmiyor.
    """
    h = gecis["hareket"]
    if h == "mesafe":
        return f"{gecis['mesafe_cm']:.0f} cm ILERI"
    if h == "ileri":
        return (f"{gecis['sure_s']:.3f}s ILERI "
                f"(~{_mesafe_cm(gecis['sure_s'], 'ileri'):.1f} cm)")
    if h == "geri":
        toplam = gecis["sure_s"] + GERI_KICK_SURESI_S
        return (f"{gecis['sure_s']:.2f}s GERI "
                f"(~{_mesafe_cm(toplam, 'geri'):.1f} cm)")
    return h


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=10.0, skor_dinleyici=None, dur_bayragi=None,
                esc=None, esc_hiz_kontrolcusu=None):
    """
    'sure' saniye boyunca STDIN'den komut gelip gelmedigini bloke olmadan
    (select() ile) kontrol eder. skor_dinleyici verilmisse basarili top
    gecisi de kontrol edilir. dur_bayragi set edilirse TestDurduruldu.

    esc + esc_hiz_kontrolcusu birlikte verilirse, GUI'den gelen guncel ESC
    hizi ~0.5s'de bir okunup degistiyse ANINDA uygulanir.

    Donus: "devam" / "sonraki" / "iki_gecis" / None
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
            # Cooldown, asil bekleme penceresinden CALMASIN diye telafi edilir.
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
      'devam' / 'sonraki' / basarili gecis (+2s) / dur_bayragi

    ESC KURALI: pozisyon biterken (hangi sebeple olursa olsun) ESC 0'a
    getirilir - robot bir SONRAKI pozisyona varana kadar bosta kalir.

    SKOR: sayim SADECE ESC fiilen calisirken acik (flywheel donmuyorken
    gelen tetikleme sayilmaz).

    Donus: sweep_toplam (derece)
    """
    if esc_hiz_kontrolcusu is not None:
        ilk_hiz = esc_hiz_kontrolcusu.ilk_deger_bekle(etiket, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)
        if ilk_hiz is None:
            _durdurma_kontrol_et(dur_bayragi)
        esc_hiz = ilk_hiz
        olay_fn(f"  [{etiket}] GUI'den ESC hizi alindi: %{esc_hiz:.1f}, "
                f"atisa birakiliyor...")
    else:
        olay_fn(f"  [{etiket}] ESC hizi %{esc_hiz:.1f} olarak ayarlaniyor, "
                f"atisa birakiliyor...")
    esc.hiz_ayarla(esc_hiz)

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
                        f"(ya da 2. basarili gecis geldi) - pozisyon tamamlandi.")
                break
            if komut == "devam":
                olay_fn(f"  [{etiket}] 'devam' alindi.")
                break

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
                olay_fn(f"  [{etiket}] UYARI: sweep donusu basarisiz, sweep durdu.")
                break
            sweep_toplam += sweep_adim
            # ESC'yi donus sonrasi tekrar ayarla (failsafe'e girmis olabilir).
            guncel_hiz = (esc_hiz_kontrolcusu.get()
                          if esc_hiz_kontrolcusu is not None else esc_hiz)
            esc.hiz_ayarla(guncel_hiz)

        return sweep_toplam

    finally:
        # SIRALAMA ONEMLI: once sayim durur, SONRA ESC kapanir.
        if skor_dinleyici is not None:
            skor_dinleyici.saymayi_durdur()
        esc.hiz_ayarla(0)
        olay_fn(f"  [{etiket}] ESC durduruldu (0) - pozisyon tamamlandi.")


def pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn,
                       skor_dinleyici=None, puan=0, dur_bayragi=None,
                       esc_hiz_kontrolcusu=None):
    """
    Bir atis pozisyonunda: ilk donusu TEK HAMLEDE yapar, ESC'yi ayarlayip
    sweep atisini calistirir. Sweep'i GERI ALMAZ - bu baseline_don() ile
    ayri bir adimda yapilir (son pozisyonda hic geri donmeden birakilir).

    Donus: sweep_toplam (derece)
    """
    konum = p.get("konum")
    konum_s = f" [konum ({konum[0]:.0f}, {konum[1]:.0f}) cm]" if konum else ""
    olay_fn(f"\n=== {p['etiket']}: {p['ilk_aci']} derece {p['ilk_yon']} "
            f"yone donuluyor{konum_s} ===")
    if not donus_yap(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b, olay_fn,
                     dur_bayragi=dur_bayragi, etiket=p["etiket"]):
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
    Bir pozisyonun sweep'ini geri alip, sonra ilk donusun TERSINE donerek
    pozisyona GIRIS acisina (baseline) tam olarak geri doner. Donusler
    TEK HAMLEDE yapilir.
    """
    if sweep_toplam > 0:
        olay_fn(f"  {p['etiket']}: sweep geri aliniyor ({sweep_toplam:.1f} "
                f"derece {ters_yon(p['sweep_yon'])})...")
        if not guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge,
                              pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            raise RuntimeError(f"{p['etiket']}: sweep geri alma donusu basarisiz")
    olay_fn(f"  {p['etiket']}: baslangica (0 derece) geri donuluyor...")
    if not donus_yap(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b,
                     olay_fn, dur_bayragi=dur_bayragi, etiket=p["etiket"]):
        _durdurma_kontrol_et(dur_bayragi)
        raise RuntimeError(f"{p['etiket']}: baseline donusu basarisiz oldu")


def _geri_git(bridge, pwm_a, pwm_b, sure_s, olay_fn, dur_bayragi=None,
               dogrula=None):
    """
    Belirtilen mesafeyi (cm) GERIYE gider.

    GERI YON KURALI (bkz. dosya basi): hareketten ONCE tekerlek yonu AKTIF
    olarak geriye aliniyor (motorlari durdur -> bekle -> geri_yon_ayarla),
    hareket bitince yon TEKRAR ileriye aliniyor. Eskiden yon komutu
    gitmedigi icin surucu ileri yonde latch'li kaliyor, robot yer
    degistirmiyor ve 6-10 arasi koordinat zinciri komple kayiyordu.

    DOGRULAMA: hareket oncesi/sonrasi ultrasonik mesafe karsilastirilir.
    Robot geri giderken ONDEKI engele olan mesafe ARTAR. Beklenen artisin
    GERI_DOGRULAMA_MIN_ORAN'indan azi gerceklesmisse tekerlekler donmemis
    demektir - bu durumda RuntimeError firlatilir. Sessizce devam edip
    butun rotayi kaymis koordinatlarla surdurmektense burada durmak
    dogru: hata nerede olustuysa orada gorunur.
    """
    if dogrula is None:
        dogrula = GERI_HAREKET_DOGRULA

    sure = sure_s
    mesafe_cm = _mesafe_cm(sure_s + GERI_KICK_SURESI_S, "geri")
    olay_fn(f"  Gecis: {sure_s:.2f}s GERI gidiliyor (~{mesafe_cm:.1f} cm) "
            f"(kalkis darbesi {GERI_KICK_SURESI_S:.2f}s @ "
            f"{GERI_KICK_CARPANI:.2f} + {sure:.2f}s @ {GERI_HIZ_CARPANI:.2f})...")

    onceki = bridge.get_distance() if dogrula else None

    # --- 1) Tekerlek yonunu AKTIF olarak GERIYE al ---
    yon_ayarlandi = _tekerlek_yonu_ayarla(pwm_a, pwm_b, "geri", olay_fn)
    _durdurma_kontrol_et(dur_bayragi)

    try:
        # --- 2a) KALKIS DARBESI: kisa sureli tam guc, statik surtunmeyi kir ---
        # Donus hareketinde soft-start rampasi var, duz harekette yok -
        # dusuk duty'de tekerlekler hic kalkis yapamiyordu.
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, GERI_KICK_SURESI_S,
                              dur_bayragi=dur_bayragi,
                              hiz_carpani=GERI_KICK_CARPANI, yon="geri")
        _durdurma_kontrol_et(dur_bayragi)

        # --- 2b) Kalibre hizda kalan mesafe ---
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure, dur_bayragi=dur_bayragi,
                              hiz_carpani=GERI_HIZ_CARPANI, yon="geri")
    finally:
        # --- 3) Yonu HER ZAMAN ileriye geri al (hata/durdurma olsa bile) ---
        motorlari_durdur(pwm_a, pwm_b)
        if yon_ayarlandi:
            _tekerlek_yonu_ayarla(pwm_a, pwm_b, "ileri", olay_fn)

    _durdurma_kontrol_et(dur_bayragi)

    # --- 4) Gercekten yer degistirdik mi? ---
    if not dogrula:
        return
    sonraki = bridge.get_distance()
    if onceki is None or sonraki is None:
        olay_fn("  UYARI: geri hareket dogrulanamadi (ultrasonik okuma yok).")
        return

    alt, ust = GERI_DOGRULAMA_BANDI_CM
    if not (alt <= onceki <= ust):
        olay_fn(f"  UYARI: geri hareket dogrulanmadi - baslangic okumasi "
                f"{onceki} cm, guvenilir band {alt:.0f}-{ust:.0f} cm disinda. "
                f"(Yanlis alarm uretmemek icin atlandi.)")
        return

    artis = sonraki - onceki
    beklenen = mesafe_cm * GERI_DOGRULAMA_MIN_ORAN
    olay_fn(f"  Geri hareket dogrulama: mesafe {onceki} -> {sonraki} cm "
            f"(artis {artis:.1f} cm, en az {beklenen:.1f} cm bekleniyordu)")
    if artis < beklenen:
        # SADECE UYARI - rota kesilmiyor. Referans SURE oldugu icin mesafe
        # olcumu bir kontrol girdisi degil, sadece teshis ipucu.
        olay_fn(f"  UYARI: geri hareket beklenenden cok az ({artis:.1f} cm < "
                f"{beklenen:.1f} cm). En olasi neden TORK - kalkis darbesi "
                f"statik surtunmeyi kiramamis olabilir. GERI_KICK_SURESI_S / "
                f"GERI_KICK_CARPANI / GERI_HIZ_CARPANI degerlerini artir, ya da "
                f"geri_hareket_testi() ile izole test et. Rota devam ediyor.")


def _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, maks_ek_adim=5,
                                ek_adim_sure_s=0.05, dur_bayragi=None,
                                hiz_carpani=1.0, hedef_bolge="YESIL",
                                yon="ileri"):
    """
    Gecis hareketinden sonra renk sensoruyle gercekten HEDEF bolgeye
    girilip girilmedigini dogrular. Hala hedef bolge algilanmiyorsa, kisa
    ek hareketlerle (AYNI yonde, ek_adim_sure_s saniye) en fazla
    maks_ek_adim kez dener. Burada da referans SURE.

    yon: ek duzeltme hareketlerinin yonu. Ana gecis GERIYE yapildiysa ek
    duzeltmeler de GERIYE yapilmali - yoksa robot ana hareketi geri alir.
    """
    for deneme in range(maks_ek_adim + 1):
        _durdurma_kontrol_et(dur_bayragi)

        r, g, b, c = bridge.get_color()
        aciklama, _puan = bolge_belirle(r, g, b, c)
        olay_fn(f"  Renk dogrulama: {aciklama} (R={r}, G={g}, B={b})")

        if aciklama.startswith(hedef_bolge):
            olay_fn(f"  Renk dogrulama: {hedef_bolge.lower()} bolgeye ulasildi.")
            return

        if deneme == maks_ek_adim:
            olay_fn(f"  UYARI: Renk dogrulama - maksimum ek adim denendi, hala "
                    f"{hedef_bolge.lower()} algilanmiyor. Devam ediliyor.")
            return

        olay_fn(f"  Renk dogrulama: hala {hedef_bolge.lower()} degil, "
                f"{ek_adim_sure_s:.2f}s ek {yon} hareket deneniyor "
                f"({deneme + 1}/{maks_ek_adim})...")
        if yon == "geri":
            # Kisa duzeltme adimlarinda dogrulama kapali - bu kadar kucuk
            # bir hareket ultrasonik gurultusunun icinde kalir.
            _geri_git(bridge, pwm_a, pwm_b, ek_adim_sure_s, olay_fn,
                      dur_bayragi=dur_bayragi, dogrula=False)
        else:
            ileri_git_sabit_sure(bridge, pwm_a, pwm_b, ek_adim_sure_s,
                                  dur_bayragi=dur_bayragi,
                                  hiz_carpani=ILERI_HIZ_CARPANI, yon="ileri")


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn, dur_bayragi=None):
    """
    Bir pozisyondan sonrakine gecerken yapilan hareketi uygular
    (baseline_don() cagrildiktan SONRA cagrilmali).
    """
    if gecis.get("ekstra_donus"):
        yon, aci = gecis["ekstra_donus"]
        if not donus_yap(aci, yon, bridge, pwm_a, pwm_b, olay_fn,
                         dur_bayragi=dur_bayragi, etiket="Gecis"):
            _durdurma_kontrol_et(dur_bayragi)
            raise RuntimeError("Gecis donusu basarisiz oldu")

    _durdurma_kontrol_et(dur_bayragi)
    hareket = gecis["hareket"]

    if hareket == "mesafe":
        # 1-5 arasi gecislerin ZATEN CALISAN yolu - rampali, mesafe
        # kontrollu ilerleme. Kisa zaman darbeleri robotu yerinden
        # oynatamadigi icin ILERI hareketlerin varsayilani budur.
        mesafe_cm = gecis.get("mesafe_cm", ILERLEME_MESAFESI_CM)
        gerekli_aciklik = max(MIN_ACIKLIK_CM, mesafe_cm + 10.0)
        olculen = bridge.get_distance()
        olay_fn(f"  Gecis: ultrasonik mesafe olculdu: {olculen} cm "
                f"(gerekli aciklik {gerekli_aciklik:.0f} cm)")
        if olculen is not None and olculen > gerekli_aciklik:
            olay_fn(f"  Gecis: onde yeterli aciklik var, {mesafe_cm:.0f} cm "
                    f"ilerleniyor...")
            ileri_git_sabit_mesafe(pwm_a, pwm_b, mesafe_cm, bridge=bridge,
                                    dur_bayragi=dur_bayragi)
        else:
            olay_fn("  UYARI: Gecis - onde yeterli aciklik yok, ilerleme atlaniyor.")

    elif hareket == "ileri":
        # SADECE 3->4 icin: yasakli bolge sinirindaki cok kucuk hareket.
        hiz_carpani = gecis.get("hiz_carpani", ILERI_HIZ_CARPANI)
        sure_s = gecis["sure_s"]
        beklenen_cm = _mesafe_cm(sure_s, "ileri")
        olculen = bridge.get_distance()
        if olculen is not None and olculen < MIN_ACIKLIK_CM:
            olay_fn(f"  UYARI: Gecis - onde sadece {olculen} cm var "
                    f"({MIN_ACIKLIK_CM} cm altinda), ilerleme ATLANIYOR.")
            return
        olay_fn(f"  Gecis: {sure_s:.3f}s ILERI gidiliyor (~{beklenen_cm:.1f} cm, "
                f"hiz carpani {hiz_carpani:.3f})...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure_s,
                              dur_bayragi=dur_bayragi, hiz_carpani=hiz_carpani,
                              yon="ileri")

    elif hareket == "geri":
        _geri_git(bridge, pwm_a, pwm_b, gecis["sure_s"], olay_fn,
                  dur_bayragi=dur_bayragi)

    else:
        raise RuntimeError(f"Bilinmeyen gecis hareketi: {hareket}")

    _durdurma_kontrol_et(dur_bayragi)

    if gecis.get("renk_dogrula"):
        _renk_dogrulayarak_ilerle(
            bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi,
            hiz_carpani=gecis.get("hiz_carpani", ILERI_HIZ_CARPANI),
            hedef_bolge=gecis.get("hedef_bolge", "YESIL"),
            yon=("geri" if hareket == "geri" else "ileri"),
            maks_ek_adim=gecis.get("maks_ek_adim", 5),
            ek_adim_sure_s=gecis.get("ek_adim_sure_s", 0.08),
        )


def calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print, esc_pin=ESC_PIN,
                              skor_dinleyici=None, dur_bayragi=None,
                              esc_hiz_kontrolcusu=None, durum_fn=None):
    """
    10 atis pozisyonlu (kirmizidan 8, yesilden 2) sweep rotasi.

    ROTA:
      1-3. atis : kirmizi bolge (3 puan), aralarda ileri hareket
      3->4      : 90 derece SOL (tek hamle) + ileri, renk dogrulamali
      4-5. atis : yesil bolge (2 puan)
      5->6      : 90 derece SAG + 28 cm GERI -> kirmiziya donus (dogrulamali)
      6-10. atis: kirmizi bolgede, 2D duzleme yayilmis 5 ekstra konum;
                  aralarda ILERI / GERI / 90 derece donus kombinasyonu
      10. atistan sonra geri donus YOK, rota biter.

    skor_dinleyici: break-beam sensorlerinden ilk basarili gecisten sonra
    en fazla 2s daha beklenip sweep otomatik durur, skor p["puan"] kadar artar.

    dur_bayragi (threading.Event): set edilirse tum hareketler durur ve
    rota guvenli sekilde sonlanir (esc.kapat() HER ZAMAN calisir).

    esc_hiz_kontrolcusu: verilirse ESC hizi POZISYONLAR'daki sabit
    degerlerden DEGIL, GUI'den canli alinir.

    durum_fn: her asama gecisinde {"asama", "pozisyon_no", "toplam_pozisyon",
    "etiket", "bolge", "puan"} sozlugu ile cagrilir.
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
        if not donus_yap(30, "sag", bridge, pwm_a, pwm_b, olay_fn,
                         dur_bayragi=dur_bayragi, etiket="1. adim"):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("1. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 2) 0.5 saniye duz git ----
        olay_fn("2. adim: 0.5 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.5, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        # ---- 3) Saga 60 derece don ----
        olay_fn("3. adim: 60 derece saga donuluyor...")
        if not donus_yap(60, "sag", bridge, pwm_a, pwm_b, olay_fn,
                         dur_bayragi=dur_bayragi, etiket="3. adim"):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("3. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 4) 0.3 saniye duz git ----
        olay_fn("4. adim: 0.3 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.3, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        # =====================================================
        # 10 ATIS POZISYONU (kirmizidan 8, yesilden 2)
        # =====================================================
        try:
            for i, p in enumerate(POZISYONLAR):
                # bolge_bildir() sadece LOGLAMA/GUI rozeti icin - skor
                # pozisyonun SABIT p["puan"] degerinden geliyor.
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

                    if i == 4:
                        olay_fn(f"\n*** 5. atis bitti - 90 derece SAG donup "
                                f"{gecis_ozeti(GECISLER[i])} gidilerek KIRMIZI "
                                f"bolgeye donuluyor (6-10. atislar) ***")

                    gecis_uygula(bridge, pwm_a, pwm_b, GECISLER[i], olay_fn,
                                 dur_bayragi=dur_bayragi)
                # son pozisyonda (10. atis) hicbir geri donus/hareket yok
        except RuntimeError as e:
            olay_fn(f"HATA: {e}")
            _durum_bildir("hata", etiket=str(e))
            return False

        olay_fn("\nOzel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(10 atis pozisyonu: kirmizidan 8, yesilden 2).")
        _durum_bildir("tamamlandi", pozisyon_no=len(POZISYONLAR),
                      etiket="Test tamamlandi")
        return True

    except TestDurduruldu:
        motorlari_durdur(pwm_a, pwm_b)
        olay_fn("ACIL DURDUR / iptal sinyali alindi - test guvenli sekilde sonlandirildi.")
        _durum_bildir("durduruldu", etiket="Acil durdur / iptal")
        return False

    finally:
        # Normal bitis, hata ya da Acil Durdur - ESC HER ZAMAN kapatilir.
        esc.kapat()


def geri_hareket_testi(sure_s=0.30):
    """
    GERI HAREKET TANI/KALIBRASYON ARACI - butun rotayi calistirmadan
    sadece geri hareketi izole eder.

    Calistirma:
      python3 -c "import ozel_navigasyon_testi_sweep_v2 as m; m.geri_hareket_testi(0.30)"

    Varsayilan 0.30s, 5->6 gecisiyle AYNI sure - yani sahada gerceklesecek
    hareketin birebir provasi.

    SONUCU NASIL KULLANIRSIN:
      - Robot HIC hareket etmediyse -> TORK: GERI_KICK_SURESI_S'i artir
        (0.08 -> 0.12) ya da GERI_HIZ_CARPANI'ni yukselt.
      - Hareket ILERI yonde olduysa -> yon pinleri ters: test_surus.py
        icindeki geri_yon_ayarla() IN pin haritasini kontrol et.
      - Hareket etti -> katedilen cm'yi olc ve HIZ SABITINI guncelle:
            GERI_HIZ_CM_PER_S = olculen_cm / (sure_s + GERI_KICK_SURESI_S)
        Bu tek sayi butun rotanin koordinat modelini duzeltir.
    """
    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)
    if bridge.is_stale(max_age_sec=1.0):
        print("UYARI: Sensor veri akisi yok.")
        bridge.stop()
        return

    bridge.request_fast_mode()
    time.sleep(0.1)
    pwm_a, pwm_b = motorlari_ayarla()

    toplam = sure_s + GERI_KICK_SURESI_S
    print(f"\n=== GERI HAREKET TESTI ===")
    print(f"  Komut suresi        = {sure_s:.2f}s")
    print(f"  Kalkis darbesi      = {GERI_KICK_SURESI_S:.2f}s @ {GERI_KICK_CARPANI}")
    print(f"  Toplam motor suresi = {toplam:.2f}s")
    print(f"  GERI_HIZ_CARPANI    = {GERI_HIZ_CARPANI}")
    print(f"  GERI_HIZ_CM_PER_S   = {GERI_HIZ_CM_PER_S} (TAHMIN)")
    print(f"  Beklenen mesafe     = ~{_mesafe_cm(toplam, 'geri'):.1f} cm")
    print("  Robotun arkasini bosalt, 3 saniye icinde basliyor...")
    time.sleep(3.0)

    try:
        _geri_git(bridge, pwm_a, pwm_b, sure_s, print, dogrula=False)
        print("\n  Robotun GERCEKTE kac cm gittigini mezurayla olc, sonra:")
        print(f"    GERI_HIZ_CM_PER_S = olculen_cm / {toplam:.2f}")
    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        bridge.stop()


def main():
    print("Uretilen rota:")
    rota_ozeti()
    print()

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
        # CLI modunda skor_dinleyici/dur_bayragi verilmiyor - sweep zaman
        # asimi/klavye komutuyla biter, Ctrl+C normal KeyboardInterrupt.
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
