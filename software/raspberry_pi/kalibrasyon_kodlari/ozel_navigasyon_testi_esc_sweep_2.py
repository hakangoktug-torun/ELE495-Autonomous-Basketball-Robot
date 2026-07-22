"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

ozel_navigasyon_testi_esc.py'nin ayni iskeletini kullanir (1-4. adimlar
ayni: 30 derece sag -> 0.5s duz -> 60 derece sag -> 0.3s duz), ama HER
atis pozisyonunda kullanicidan yon/aci SORMAK yerine SABIT bir baslangic
acisiyla doner, ESC'yi calistirir, sonra kullanicidan komut gelene kadar
3'er derecelik adimlarla ACISAL TARAMA (sweep) yaparak ayni bolgeye
farkli acilardan atmayi dener.

GUNCELLEMELER (bu surum):
  1) SKOR DINLEYICI: sweep dongusu artik sadece klavye komutuyla
     ('devam'/'sonraki') degil, break-beam sensorlerinden (Arduino R4
     WiFi -> Flask /skor route -> SkorDinleyici) gelen "top cemberden
     gecti" bilgisiyle de bitebiliyor. Bir pozisyonda 2 basarili gecis
     sayilinca sweep otomatik durup bir sonraki pozisyona geciliyor.
     GUI'deki skor da (bolge_bildir'in dondugu puan uzerinden) her
     gecişte aninda artiyor.
  2) ACIL DURDUR: tum hareket fonksiyonlarina (guvenli_donus,
     ileri_git_sabit_mesafe/sure) bir dur_bayragi (threading.Event)
     gecirilebiliyor. Bu bayrak set edildiginde (Acil Durdur butonu ya
     da Ctrl+C), calisan TUM hareketler en kisa surede durduruluyor ve
     TestDurduruldu istisnasi firlatilarak butun rota GUVENLI sekilde
     ve ANINDA sonlandiriliyor (finally: esc.kapat() her zaman calisir).
  3) SWEEP BEKLEME SURESI: her sweep adiminda beklenen sure 20s'den
     10s'ye dusuruldu (POZISYON_1..6 icindeki sweep_bekleme degerleri).
  4) 2. POZISYON ACISI: 90 dereceden 93 dereceye guncellendi.
  5) KIRMIZI->YESIL GECISI (3->4, GECISLER[2]): artik SABIT SURE ile
     DEGIL, ultrasonik mesafe sensoruyle - onde bir engele (sahanin
     kenari/duvari) 35cm kalana kadar ilerliyor. Boylece robotun her
     zaman NEREDE durdugu (yaklasik olarak) bilinir, zamanlamaya bagli
     tahmine gerek kalmaz. Vardiktan sonra hala renk sensoruyle
     DOGRULANIYOR - yesil algilanmiyorsa kisa ek ileri hareketlerle (en
     fazla 3 kez) tekrar denenir.

AMAC: Robotun donuslerde birkac derecelik sapma yasayabilmesi yuzunden
("tam 90 dönemiyor" sorunu) TEK bir sabit aciya guvenmek yerine, o
acinin etrafindaki birkac komsu aciyi da otomatik deneyerek potansiyel
sapmanin sonucunu (top cemberden gecmemesi) telafi etmeye calisir.

POZISYONLAR (6 atis: kirmizidan 3, yesilden 3):
  1. atis (Kirmizi): 87 derece SOL, ESC %12.4, sweep SAGA, maks 3 tekrar.
  2. atis (Kirmizi, YENI): 93 derece SOL, ESC %12.4, sweep SAGA, maks 3.
  3. atis (Kirmizi): 90 derece SOL, ESC %12.4, sweep SAGA, maks 3.
  4. atis (Yesil): 10 derece SOL, ESC %11, sweep SAGA, maks 3.
  5. atis (Yesil): 100 derece SAG, ESC %11, sweep SOLA (yon TERS!), maks 2.
  6. atis (Yesil, YENI): 105 derece SAG, ESC %11, sweep SAGA, maks 3.
     Bu pozisyondan sonra robot GERI DONMEZ, kod biter.

GECISLER (pozisyonlar arasi hareket, HER ZAMAN once baseline'a - yani
konuma varilan giris acisina - tam geri donulerek yapilir):
  1->2, 2->3: sadece 10cm ileri (donus yok, ayni kirmizi bolge icinde).
  3->4 (KIRMIZI -> YESIL): 90 derece sola donup 0.6 saniye duz gidilir
       (0.4'ten 0.6'ya cikarildi), sonra renk sensoruyle yesile
       girildigi dogrulanir.
  4->5: ekstra 90 derece sola donup 10cm ileri gidilir.
  5->6 (YENI): ekstra 90 derece sola donup 10cm ileri gidilir - ayni
       4->5 gecisiyle ayni sablon (VARSAYIM - kullanicidan "90 sola
       donup 10cm duz git" denildi, baseline'a once donulup donulmedigi
       acikca belirtilmedi; tutarlilik icin diger gecislerdeki gibi
       once baseline'a donuluyor kabul edildi).

       NOT (5->6 gecisindeki "beklenmedik sola donus" hakkinda): bu
       gecisten HEMEN once, 5. pozisyonun sweep'i (sol yonde) ve ilk
       donusu (sag 100 derece) GERI ALINIYOR - yani baseline_don()
       icinde robot ONCE (kucuk) bir SAG donusuyle sweep'i geri alir,
       SONRA (buyuk, ~100 derece) bir SOL donusle ilk donusu geri alip
       baseline'a doner. Bu buyuk SOL donus, GECISLER[4]'teki ekstra SAG
       90 derece donusten ONCE gerceklesir - yani "5. pozisyondan
       cikarken once buyuk bir sola donus, sonra saga donus" sirasi
       algoritmanin BASELINE'A DONME mantigindan kaynaklanan, planli bir
       davranistir - GECISLER listesindeki bir hata degildir. Eger bu
       istenen davranis DEGILSE (yani gercekten sadece "saga 90 don"
       gecisi isteniyorsa, baseline'a hic donmeden), GECISLER[4]'e
       "baseline_atla": True gibi bir bayrak eklenip
       calistir_ozel_rota_sweep icindeki akis ona gore
       degistirilebilir - bu, kodun mevcut halinde YAPILMADI (davranis
       degisikligi net onaylanmadan mevcut, calisir durumdaki mantik
       korundu). Terminal ciktisini (bu gecis sirasindaki donus
       loglarini) paylasirsan tam olarak hangi donusun "beklenmedik"
       geldigini birlikte teyit edebiliriz.

SURE BUTCESI (5 dakikalik demo siniri icin kaba tahmin): her sweep
adimi 10s bekleme + donus suresi demek (eskiden 20s'ydi). 1/2/3/4.
pozisyonlarda maks 3 tekrar (~33s'ye kadar her biri), 5. pozisyonda
maks 2 (~22s), 6. pozisyonda maks 3 (~33s). Kullanicinin 'sonraki'/
'atla' ile beklemeyi kisaltmasi ya da 'devam' ile pozisyonu erken
bitirmesi bu sureyi daha da azaltir. Break-beam sensorlerinden 2
basarili gecis gelmesi de ayni sekilde bekleme suresini kisaltir.

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


# ---------- Pozisyon tanimlari (deneysel - kolay ayarlanabilir) ----------
# NOT: sweep_bekleme 20.0 -> 10.0 (kullanici istegiyle kisaltildi).
POZISYON_1 = dict(ilk_yon="sol", ilk_aci=87.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="1. atis")
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=93.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="2. atis")
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=100.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="3. atis")
POZISYON_4 = dict(ilk_yon="sol", ilk_aci=10.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="4. atis")
POZISYON_5 = dict(ilk_yon="sag", ilk_aci=100.0, esc_hiz=11.0,
                   sweep_yon="sol", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=2, etiket="5. atis")
POZISYON_6 = dict(ilk_yon="sag", ilk_aci=105.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="6. atis")

POZISYONLAR = [POZISYON_1, POZISYON_2, POZISYON_3, POZISYON_4, POZISYON_5, POZISYON_6]

# ---------- Pozisyonlar arasi gecisler ----------
# GECISLER[i] = POZISYONLAR[i] -> POZISYONLAR[i+1] arasi hareket.
# Her gecis, HER ZAMAN once ilgili pozisyonun baseline'ina (giris acisina)
# tam geri donulduktan SONRA uygulanir (baseline_don() cagrisi ile).
#   ekstra_donus: (yon, derece) ya da None - baseline'a dondukten sonra
#                 EK olarak yapilan bir donus (orn. bolgeler arasi 90 derece)
#   hareket: "mesafe" -> ultrasonik mesafe kontrolu yapip ILERLEME_MESAFESI_CM
#                        kadar ilerler (aciklik yoksa atlar)
#            "sure"   -> sure_s saniye boyunca sabit sureli duz gider
#                        (zamanlamaya dayanir, mesafe olcumu YOK)
#            "engel"  -> esik_cm mesafesindeki bir engele (sahanin kenari/
#                        duvari) kadar ilerler (ileri_git_engel_bulunca ile,
#                        ultrasonik geri bildirimle - "nereye vardigimiz"
#                        kesin olarak bilinir, zamanlamaya guvenilmez)
#   renk_dogrula: True ise, "sure"/"engel" hareketinden SONRA renk sensoruyle
#                 gercekten YESIL bolgeye girilip girilmedigi kontrol
#                 edilir; degilse kisa ek ileri hareketlerle (en fazla
#                 3 kez) tekrar denenir.
GECISLER = [
    dict(ekstra_donus=None, hareket="mesafe"),                                          # 1 -> 2
    dict(ekstra_donus=None, hareket="mesafe"),                                          # 2 -> 3
    dict(ekstra_donus=("sol", 90.0), hareket="engel", esik_cm=45.0, renk_dogrula=True), # 3 -> 4 (kirmizi->yesil, sure yerine 45cm engel)
    dict(ekstra_donus=("sol", 90.0), hareket="mesafe"),                                # 4 -> 5
    dict(ekstra_donus=("sag", 90.0), hareket="mesafe"),                                # 5 -> 6 (YENI, varsayim) başta soldu
]


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=10.0, skor_dinleyici=None, dur_bayragi=None):
    """
    'sure' saniye boyunca STDIN'den bir komut gelip gelmedigini, TAM
    'sure' kadar bloke olmadan (select() ile non-blocking) kontrol eder.
    Ayrica skor_dinleyici verilmisse, o pozisyonda 2 basarili top gecisi
    olup olmadigini da periyodik kontrol eder. dur_bayragi set edilirse
    TestDurduruldu firlatir (Acil Durdur / Ctrl+C).

    Uc farkli komut ayirt edilir:
      'devam'     -> kullanici bu POZISYONU tamamen bitirmek istiyor
      'sonraki' (ya da 'atla')
                  -> kullanici sadece bu sweep adimini erken bitirip HEMEN
                     bir sonraki sweep acisina gecmek istiyor - pozisyon
                     bitmiyor, kalan bekleme suresi atlaniyor
      2 basarili gecis (skor_dinleyici uzerinden)
                  -> bu pozisyonda break-beam sensorleri topun cemberden
                     2 kez gectigini algiladi, pozisyon OTOMATIK bitirilmeli

    Donus: "devam"     -> kullanici pozisyonu bitirmek istedi
           "sonraki"   -> kullanici bir sonraki sweep acisina hemen gecmek istedi
           "iki_gecis" -> 2 basarili top gecisi algilandi, pozisyon bitirilmeli
           None        -> sure doldu, hicbir sey olmadi (normal akis, sweep'e devam)
    """
    bitis = time.time() + sure
    while True:
        _durdurma_kontrol_et(dur_bayragi)

        kalan = bitis - time.time()
        if kalan <= 0:
            return None

        if skor_dinleyici is not None and skor_dinleyici.iki_gecis_oldu_mu():
            return "iki_gecis"

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
                    skor_dinleyici=None, dur_bayragi=None):
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

    Hicbir komut/gecis gelmezse (sure dolarsa) normal akista bir sonraki
    sweep adimina geciliyor - yani 'sonraki' ile zaman asiminin sonucu
    AYNI, tek fark 'sonraki' 10s beklemeden hemen tetikliyor olmasi.

    NOT: Konuma varista yapilan ILK donusu (ornegin '87 derece sol')
    CAGIRAN KOD onceden yapmis olmali - bu fonksiyon SADECE sweep
    adimlarini yapar.

    Donus: sweep_toplam (derece) - sweep boyunca sweep_yon yonunde
    TOPLAM ne kadar donuldugu. Geri donerken TERS yonde ayni miktarda
    donmen gerekir.
    """
    olay_fn(f"  [{etiket}] ESC hizi %{esc_hiz:.1f} olarak ayarlaniyor, "
            f"atisa birakiliyor...")
    esc.hiz_ayarla(esc_hiz)

    sweep_toplam = 0.0
    sweep_sayaci = 0
    while True:
        olay_fn(f"  [{etiket}] {sweep_bekleme:.0f}s bekleniyor - 2 basarili "
                f"gecis olursa ya da 'devam' yazarsan pozisyon biter, "
                f"'sonraki' (ya da 'atla') yazarsan beklemeden hemen bir "
                f"sonraki aciya gecilir...")
        komut = girdi_bekle(sweep_bekleme, skor_dinleyici=skor_dinleyici,
                             dur_bayragi=dur_bayragi)

        if komut == "iki_gecis":
            olay_fn(f"  [{etiket}] 2 basarili gecis algilandi - pozisyon "
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
        esc.hiz_ayarla(esc_hiz)

    return sweep_toplam


def pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn,
                       skor_dinleyici=None, puan=0, dur_bayragi=None):
    """
    Bir atis pozisyonunda: ilk donusu yapar, sayimi baslatir (skor_dinleyici
    verilmisse - bu pozisyonun bolge puanini kaydeder), ESC'yi ayarlayip
    sweep atisini calistirir, sonunda sayimi durdurur (hareket/gecis
    sirasinda yanlislikla sayilmasin diye).

    Sweep'i GERI ALMAZ - bu, baseline_don() ile ayri bir adimda yapilir
    (boylece son pozisyonda hic geri donmeden biraktirilabilir).

    Donus: sweep_toplam (derece, sweep_yon yonunde toplam donulen miktar)
    """
    olay_fn(f"\n=== {p['etiket']}: {p['ilk_aci']} derece {p['ilk_yon']} "
            f"yone donuluyor ===")
    if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b,
                          dur_bayragi=dur_bayragi):
        _durdurma_kontrol_et(dur_bayragi)
        raise RuntimeError(f"{p['etiket']}: ilk donus basarisiz oldu")

    if skor_dinleyici is not None:
        skor_dinleyici.saymaya_basla(puan)

    try:
        return sweep_atis_yap(
            bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
            p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
            olay_fn, p["etiket"], skor_dinleyici=skor_dinleyici,
            dur_bayragi=dur_bayragi,
        )
    finally:
        if skor_dinleyici is not None:
            skor_dinleyici.saymayi_durdur()


def baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn, dur_bayragi=None):
    """
    Bir pozisyonun sweep'ini geri alip (varsa), sonra ilk donusun
    TERSINE donerek pozisyona GIRIS acisina (baseline, 0 derece relatif)
    tam olarak geri doner.
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
    if not guvenli_donus(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b,
                          dur_bayragi=dur_bayragi):
        _durdurma_kontrol_et(dur_bayragi)
        raise RuntimeError(f"{p['etiket']}: baseline donusu basarisiz oldu")


def _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, maks_ek_adim=3,
                                adim_suresi=0.15, dur_bayragi=None):
    """
    Zamanli gecis hareketinden sonra renk sensoruyle gercekten YESIL
    bolgeye girilip girilmedigini dogrular. Hala KIRMIZI/taninmayan
    okunuyorsa, kisa (adim_suresi) ek ileri hareketlerle en fazla
    maks_ek_adim kez dener.
    """
    for deneme in range(maks_ek_adim + 1):
        _durdurma_kontrol_et(dur_bayragi)

        r, g, b, c = bridge.get_color()
        aciklama, puan = bolge_belirle(r, g, b, c)
        olay_fn(f"  Renk dogrulama: {aciklama} (R={r}, G={g}, B={b})")

        if aciklama.startswith("YESIL"):
            olay_fn("  Renk dogrulama: yesil bolgeye ulasildi.")
            return

        if deneme == maks_ek_adim:
            olay_fn("  UYARI: Renk dogrulama - maksimum ek adim denendi, hala "
                    "yesil algilanmiyor. Devam ediliyor (bolge tespiti atis "
                    "sirasinda tekrar yapilacak).")
            return

        olay_fn(f"  Renk dogrulama: hala yesil degil, {adim_suresi}s ek ileri "
                f"hareket deneniyor ({deneme + 1}/{maks_ek_adim})...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, adim_suresi, dur_bayragi=dur_bayragi)


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn, dur_bayragi=None):
    """
    Bir pozisyondan sonrakine gecerken yapilan hareketi uygular
    (baseline_don() cagrildiktan SONRA cagrilmali). GECISLER listesindeki
    bir gecis sozlugunu alir - bkz. dosya basindaki GECISLER aciklamasi.
    """
    if gecis.get("ekstra_donus"):
        yon, aci = gecis["ekstra_donus"]
        olay_fn(f"  Gecis: {aci} derece {yon} yone (ekstra) donuluyor...")
        if not guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
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
        olay_fn(f"  Gecis: {gecis['sure_s']}s duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, gecis["sure_s"], dur_bayragi=dur_bayragi)

        _durdurma_kontrol_et(dur_bayragi)

        if gecis.get("renk_dogrula"):
            _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi)
    elif gecis["hareket"] == "engel":
        # Sure yerine ULTRASONIK MESAFE ile ilerleme: onde bir engel
        # (orn. sahanin dis duvari/kenari) esik_cm mesafesine gelene kadar
        # gider. Bu, sabit-sureli harekete gore "nereye vardigimizi kesin
        # olarak biliriz" avantaji sagliyor - zamanlamaya degil, gercek
        # mesafe olcumune dayaniyor.
        esik_cm = gecis.get("esik_cm", 35.0)
        olay_fn(f"  Gecis: onde {esik_cm}cm'ye bir engel bulunana kadar "
                f"ilerleniyor...")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=esik_cm,
                                           dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)
        if not bulundu:
            olay_fn(f"  UYARI: Gecis - {esik_cm}cm'de engel bulunamadi "
                    f"(zaman asimi), guvenlik amacli durulmustu. Devam ediliyor.")

        if gecis.get("renk_dogrula"):
            _renk_dogrulayarak_ilerle(bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=dur_bayragi)


def calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print, esc_pin=ESC_PIN,
                              skor_dinleyici=None, dur_bayragi=None):
    """
    calistir_ozel_rota()'nin kullanicidan aci sorma kismini kaldirip
    yerine sabit-aci + otomatik-sweep mantigi koyan, 6 atis pozisyonlu
    (kirmizidan 3, yesilden 3) versiyonu.

    skor_dinleyici verilirse: her pozisyonda break-beam sensorlerinden
    2 basarili gecis geldiginde sweep otomatik durur ve GUI'deki skor,
    bolgenin rengine gore (bolge_bildir'in dondugu puan) aninda artar.

    dur_bayragi verilirse (threading.Event): set edildiginde (Acil Durdur
    butonu / Ctrl+C) TUM hareketler en kisa surede durdurulur ve rota
    guvenli sekilde sonlandirilir (esc.kapat() HER ZAMAN calisir).
    """
    esc = EscKontrol(pin=esc_pin)
    esc.baslat()

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
        # 6 ATIS POZISYONU (kirmizidan 3, yesilden 3)
        # =====================================================
        try:
            for i, p in enumerate(POZISYONLAR):
                _, puan = bolge_bildir(bridge, olay_fn)
                sweep_toplam = pozisyon_calistir(
                    bridge, pwm_a, pwm_b, esc, p, olay_fn,
                    skor_dinleyici=skor_dinleyici, puan=puan,
                    dur_bayragi=dur_bayragi,
                )

                son_pozisyon = (i == len(POZISYONLAR) - 1)
                if not son_pozisyon:
                    baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn,
                                 dur_bayragi=dur_bayragi)
                    gecis_uygula(bridge, pwm_a, pwm_b, GECISLER[i], olay_fn,
                                 dur_bayragi=dur_bayragi)
                # son pozisyonda (6. atis) hicbir geri donus/hareket yok
        except RuntimeError as e:
            olay_fn(f"HATA: {e}")
            return False

        olay_fn("\nOzel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(6 atis pozisyonu: kirmizidan 3, yesilden 3).")
        return True

    except TestDurduruldu:
        motorlari_durdur(pwm_a, pwm_b)
        olay_fn("ACIL DURDUR / iptal sinyali alindi - test guvenli sekilde sonlandirildi.")
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
