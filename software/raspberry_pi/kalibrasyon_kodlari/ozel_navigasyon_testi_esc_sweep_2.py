"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

ozel_navigasyon_testi_esc.py'nin ayni iskeletini kullanir (1-4. adimlar
ayni: 30 derece sag -> 0.5s duz -> 60 derece sag -> 0.3s duz), ama HER
atis pozisyonunda kullanicidan yon/aci SORMAK yerine SABIT bir baslangic
acisiyla doner, ESC'yi calistirir, sonra kullanicidan komut gelene kadar
2'şer derecelik adimlarla ACISAL TARAMA (sweep) yaparak ayni bolgeye
farkli acilardan atmayi dener.

GUNCELLEME (BU SURUM - iki duzeltme, kullanici istegiyle):
  1) GERI GIDERKEN YAMULMA DUZELTMESI (5->6 gecisi): zamanli duz gitme
     artik bu dosyadaki YENI duz_git_sabit_sure_duzeltmeli() fonksiyonuyla
     yapiliyor - BNO055 heading geri beslemeli VE YON-DUYARLI. Kritik
     nokta: heading duzeltmesinin isareti GERI giderken TERS cevrilmeli.
     Ileri giderken "saga kaydi -> sol tekeri yavaslat, sagi hizlandir"
     dogru davranis; geri giderken AYNI duzeltme yamulmayi duzeltmek
     yerine BUYUTUR (pozitif geri besleme) - robotun geri giderken
     gitgide artan bir kavisle sapmasinin sebebi buydu. Yeni fonksiyon
     yon="geri" iken isareti ters ceviriyor; boylece hem ileri hem geri
     zamanli hareketler duz cizgide tutuluyor. Renk dogrulamanin ek
     adimlari da ayni fonksiyonu kullaniyor.
  2) 6->7 VE 7. POZISYON ACI KIRPMALARI: robot bu donuslerde sistematik
     fazla donuyordu (diger acilarda yapilan kirpmalarla ayni oruntu):
     6->7 gecis donusu 70 -> 67 derece, 7. pozisyon ilk_aci 100 -> 96
     derece. Testte hala fazlaysa 1-2 derece daha kirp, eksik kalirsa
     geri artir. NOT: 8. pozisyonun 110 derecelik donusu AYNI sistematik
     hatayi tasiyorsa onu da ~106'ya kirpman gerekebilir - once test et.

ROTA (8 pozisyon - guncel acilar):
  1. atis (Kirmizi, 3p): 89 derece SOL, ESC %11.3.
  1->2 : sweep geri, hedef acidan 89 derece SAG, 10cm ileri.
  2. atis (Kirmizi, 3p): 104 derece SOL, ESC %11.3.
  2->3 : sweep geri, hedef acidan 104 derece SAG, 10cm ileri.
  3. atis (Kirmizi, 3p): 112 derece SOL, ESC %11.4.
  3->4 : sweep geri, hedef acidan 22 derece SAG, 0.075s ileri, yesil dogrulamali.
  4. atis (Yesil, 2p): 38 derece SOL, ESC %9.8.
  4->5 : sweep geri, hedef acidan 52 derece SOL (38+52=90 net), 10cm ileri.
  5. atis (Yesil, 2p): 84 derece SAG, ESC %9.8.
  5->6 : sweep geri, hedef aciya 6 derece SAG eklenir (84+6=90 net),
         duraksamadan 1.2s DUZ GERI (heading duzeltmeli). Kirmizi dogrulanir.
  6. atis (Kirmizi, 3p): 2 derece SOL, ESC %11.5.
  6->7 : sweep geri, hedef acidan 92 derece SOL, 0.8s ILERI.
  7. atis (Kirmizi, 3p): 115 derece SOL, ESC %11.4.
  7->8 : sweep geri, hedef acidan 65 derece SOL, 0.8s ILERI.
  8. atis (Kirmizi, 3p): 100 derece SAG, ESC %11.5 - SON POZISYON, rota biter.

ONEMLI TASARIM NOTLARI:
  - GECIS MODU "sadece_sweep_geri": bu bayragi tasiyan gecislerde sweep
    acilari geri alinir ama pozisyonun ilk_aci donusu GERI ALINMAZ -
    robot ilk HEDEF ACIDA kalir ve gecisin ekstra_donus'u bu hedef
    acinin UZERINE eklenir. ("baseline_atla" bayragi da hala
    destekleniyor - o hicbir seyi geri almaz.)
  - TEK PARCA DONUSLER: tum bolme mantiklari (100+ ikiye bolme, 90'i
    45+45 yapma) kaldirilmis durumda - her donus tek hamlede.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(ozel_navigasyon_testi_esc.py, donus_kapali_dongu.py, robot_bridge.py,
test_surus.py, donus_hassas.py, atici_esc_kontrol_pigpio_2.py ve
skor_dinleyici.py ile ayni yerde olmali)
"""

import sys
import os
import select
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT, aci_farki
from donus_hassas import guvenli_donus
from test_surus import (
    ileri_git_sabit_mesafe, ileri_git_engel_bulunca,
    ileri_yon_ayarla, geri_yon_ayarla, SOL_HIZ, SAG_HIZ,
)
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


# ---------- Duzeltmeli duz gitme (YENI - "geri giderken yamulma" cozumu) ----------
DUZ_KAZANC_P = 0.4        # derece basina duty duzeltme (test_surus ile ayni)
DUZ_MAKS_DUZELTME = 10.0  # duty cinsinden toplam duzeltme ust siniri


def duz_git_sabit_sure_duzeltmeli(bridge, pwm_a, pwm_b, sure_s, yon="ileri",
                                    hiz_carpani=1.0, dur_bayragi=None):
    """
    BNO055 heading geri beslemeli, sabit sureli DUZ gitme - hem ILERI hem
    GERI yonde dogru calisir. Gecislerdeki "sure" hareketlerinde
    ozel_navigasyon_testi_esc.ileri_git_sabit_sure'un YERINE kullanilir.

    NEDEN GEREKTI ("geri giderken yamulan cizgi" duzeltmesi): heading
    duzeltmesinin isareti, GERI giderken TERS cevrilmelidir. Ileri
    giderken "heading saga kaydi -> sol tekeri yavaslat, sag tekeri
    hizlandir" robotu sola kivirir ve hatayi kapatir. GERI giderken ise
    tekerlerin yer hizlari ters isaretli oldugu icin AYNI duty degisikligi
    robotu TERS yone kivirir - yani duzeltme hatayi kapatacagina BUYUTUR
    (pozitif geri besleme). Sonuc: geri giderken gitgide artan bir kavis.
    Bu fonksiyon yon="geri" iken duzeltme isaretini ters cevirerek her iki
    yonde de duz cizgiyi korur.

    NOT: SOL_HIZ/SAG_HIZ trim degerleri ileri yon icin kalibre edilmisti -
    geri yonde surtunme/disli farklari olabilir, ama P dongusu bu kucuk
    asimetriyi zaten canli olarak telafi eder.
    """
    hedef_heading = bridge.get_heading()

    if yon == "geri":
        geri_yon_ayarla()
        isaret = -1.0   # KRITIK: geri giderken duzeltme isareti TERS
    else:
        ileri_yon_ayarla()
        isaret = 1.0

    temel_sol = SOL_HIZ * hiz_carpani
    temel_sag = SAG_HIZ * hiz_carpani
    pwm_a.ChangeDutyCycle(temel_sol)
    pwm_b.ChangeDutyCycle(temel_sag)

    baslangic = time.time()
    adim = 0
    while time.time() - baslangic < sure_s:
        if dur_bayragi is not None and dur_bayragi.is_set():
            motorlari_durdur(pwm_a, pwm_b)
            _durdurma_kontrol_et(dur_bayragi)

        simdiki = bridge.get_heading()
        if simdiki is not None and hedef_heading is not None:
            hata = aci_farki(hedef_heading, simdiki)
            duzeltme = isaret * max(-DUZ_MAKS_DUZELTME,
                                     min(DUZ_MAKS_DUZELTME, DUZ_KAZANC_P * hata))
            sol_duty = max(0, min(100, temel_sol - duzeltme))
            sag_duty = max(0, min(100, temel_sag + duzeltme))
            pwm_a.ChangeDutyCycle(sol_duty)
            pwm_b.ChangeDutyCycle(sag_duty)
            if adim % 5 == 0:
                print(f"  [DEBUG] duz({yon}): heading={simdiki} hata={hata:.1f} "
                      f"sol={sol_duty:.1f} sag={sag_duty:.1f}")
        adim += 1
        time.sleep(0.02)

    motorlari_durdur(pwm_a, pwm_b)


# ---------- Donus uygulama (BOLME KALDIRILDI - kullanici istegiyle) ----------
# ESKI DAVRANIS: 100 dereceden buyuk donusler iki esit parcaya bolunuyordu.
# KALDIRILMA SEBEBI: her ekstra durma/kalkis, robotun oldugu konumdan hafifce
# KAYMASINA yol aciyordu - iki parcali donus, tek parcaya kiyasla konum
# hatasini AZALTMAK yerine ARTIRIYORDU (donus_hassas'in erken durma + fren
# mekanizmasi tek parcada zaten yeterince hassas). Artik her aci TEK PARCA
# donuluyor.


def _buyuk_aci_donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None):
    """
    Bir donusu TEK PARCA uygular (bolme mantigi kaldirildi).

    aci <= 0 ise donus TAMAMEN ATLANIR ve True donulur. Ayni guard
    baseline donusunde de gecerli.

    Donus: True (basarili) / False (basarisiz)
    """
    if aci <= 0:
        return True  # donus yok - dogrudan basarili say

    return guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi)


# ---------- Pozisyon tanimlari (deneysel - kolay ayarlanabilir) ----------
# ESC hizlari ve acilar kullanicinin fiziksel test sonuclarina gore ayarlanmis
# guncel degerlerdir. ESC: 1-3. pozisyonlar 11.5'e, 4-5. pozisyonlar 10.1'e
# dusuruldu (kullanici istegiyle).
POZISYON_1 = dict(ilk_yon="sol", ilk_aci=87.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="1. atis", puan=3)
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=102.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="2. atis", puan=3)
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=112.0, esc_hiz=11.4,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="3. atis", puan=3)
POZISYON_4 = dict(ilk_yon="sol", ilk_aci=36.0, esc_hiz=9.8,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="4. atis", puan=2)
POZISYON_5 = dict(ilk_yon="sag", ilk_aci=84.0, esc_hiz=9.8,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="5. atis", puan=2)
POZISYON_6 = dict(ilk_yon="sol", ilk_aci=2.0, esc_hiz=11.5,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="6. atis", puan=3)
POZISYON_7 = dict(ilk_yon="sol", ilk_aci=115.0, esc_hiz=11.4,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="7. atis", puan=3)
POZISYON_8 = dict(ilk_yon="sag", ilk_aci=93.0, esc_hiz=11.5,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="8. atis", puan=3)

POZISYONLAR = [POZISYON_1, POZISYON_2, POZISYON_3, POZISYON_4, POZISYON_5,
               POZISYON_6, POZISYON_7, POZISYON_8]

# ---------- Pozisyonlar arasi gecisler ----------
# GECISLER[i] = POZISYONLAR[i] -> POZISYONLAR[i+1] arasi hareket.
#   ekstra_donus: (yon, derece) ya da None
#   hareket: "mesafe" / "sure" / "engel"
#   renk_dogrula: True ise hareket sonrasi hedef_bolge dogrulanir.
#   sadece_sweep_geri / baseline_atla: gecis oncesi donus modu (ana
#   dongudeki aciklamaya bak).
GECISLER = [
    dict(ekstra_donus=("sag", 87.0), hareket="mesafe",
         sadece_sweep_geri=True),                                                      # 1 -> 2 (GUNCELLENDI: 89->87 - sweep geri, hedef acidan 87 derece SAG, 10cm ileri)
    dict(ekstra_donus=("sag", 102.0), hareket="mesafe",
         sadece_sweep_geri=True),                                                      # 2 -> 3 (GUNCELLENDI: 104->102 - sweep geri, hedef acidan 102 derece SAG, 10cm ileri)
    dict(ekstra_donus=("sag", 22.0), hareket="sure", sure_s=0.075, renk_dogrula=True,
         hiz_carpani=1.3 / 1.5, sadece_sweep_geri=True),                               # 3 -> 4 (sweep geri, 112'lik hedef acidan 22 derece SAG, 0.075s duz ileri, yesil dogrulamali)
    dict(ekstra_donus=("sol", 54.0), hareket="mesafe",
         sadece_sweep_geri=True),                                                      # 4 -> 5 (GUNCELLENDI: 52->54 - sweep geri, 36'lik hedef acidan 54 derece SOL (36+54=90 net), 10cm ileri)
    dict(ekstra_donus=("sag", 6.0), hareket="sure", sure_s=1.2, yon="geri",
         renk_dogrula=True, hedef_bolge="KIRMIZI", sadece_sweep_geri=True),            # 5 -> 6 (sweep geri, 84'luk hedef aciya 6 derece SAG eklenir (84+6=90 net), 1.2s DUZ GERI heading duzeltmeli, kirmizi dogrulamali)
    dict(ekstra_donus=("sag", 92.0), hareket="sure", sure_s=0.8,
         sadece_sweep_geri=True),                                                      # 6 -> 7 (GUNCELLENDI: yon sol->sag - sweep geri, 2'lik hedef acidan 92 derece SAG, 0.8s ILERI)
    dict(ekstra_donus=("sol", 65.0), hareket="sure", sure_s=0.8,
         sadece_sweep_geri=True),                                                      # 7 -> 8 (GUNCELLENDI: sweep geri, 115'lik hedef acidan 65 derece SOL (75'ten guncellendi), 0.8s ILERI)
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
    gelen GUNCEL ESC hizi okunur, degistiyse aninda uygulanir.

    Donus: "devam" / "sonraki" / "iki_gecis" / None (sure doldu)
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
    ESC'yi esc_hiz'e ayarlayip atisa birakir, sonra 'devam' / 'sonraki' /
    basarili gecis / dur_bayragi durumlarindan biri gerceklesene kadar her
    sweep_bekleme saniyede bir sweep_adim derece sweep_yon yonune doner.

    ESC KURALI: pozisyon biterken (hangi sebeple olursa olsun) bu
    fonksiyon ESC'yi 0'a (kapali) getirir.

    Donus: sweep_toplam (derece) - sweep boyunca toplam donulen miktar.
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
            # durumda da bir sonraki sweep adimina gecilir.
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
            guncel_hiz = esc_hiz_kontrolcusu.get() if esc_hiz_kontrolcusu is not None else esc_hiz
            esc.hiz_ayarla(guncel_hiz)

        return sweep_toplam

    finally:
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
    Bir atis pozisyonunda: ilk donusu yapar (ilk_aci=0 ise donus ATLANIR),
    ESC'yi ayarlayip sweep atisini calistirir.

    Donus: sweep_toplam (derece, sweep_yon yonunde toplam donulen miktar)
    """
    if p["ilk_aci"] > 0:
        olay_fn(f"\n=== {p['etiket']}: {p['ilk_aci']} derece {p['ilk_yon']} "
                f"yone donuluyor ===")
    else:
        olay_fn(f"\n=== {p['etiket']}: ilk donus YOK (yon geciste ayarlandi), "
                f"dogrudan atisa geciliyor ===")
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
    TERSINE donerek pozisyona GIRIS acisina (baseline) tam geri doner.
    ilk_aci=0 pozisyonlarda geri donus adimi otomatik atlanir
    (sadece sweep geri alinir).
    """
    if sweep_toplam > 0:
        olay_fn(f"  {p['etiket']}: sweep geri aliniyor ({sweep_toplam:.1f} "
                f"derece {ters_yon(p['sweep_yon'])})...")
        if not guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b,
                              dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            raise RuntimeError(f"{p['etiket']}: sweep geri alma donusu basarisiz oldu")
    if p["ilk_aci"] > 0:
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
    bolgeye girilip girilmedigini dogrular; girilmediyse kisa ek
    hareketlerle (ayni yonde) en fazla maks_ek_adim kez dener.

    GUNCELLEME: ek hareketler de artik duz_git_sabit_sure_duzeltmeli()
    ile yapiliyor - yani GERI yonlu ek adimlar da (5->6 dogrulamasi)
    heading duzeltmeli ve yamulmadan gidiyor.
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
        duz_git_sabit_sure_duzeltmeli(bridge, pwm_a, pwm_b, adim_suresi,
                                        yon=yon, hiz_carpani=hiz_carpani,
                                        dur_bayragi=dur_bayragi)


def _donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None):
    """
    Bir gecis donusunu TEK PARCA uygular (45+45 bolme kaldirilmis
    durumda). SADECE GECISLER'deki ekstra_donus donusleri icin
    kullaniliyor.

    Donus: True / False
    """
    olay_fn(f"  Gecis: {aci} derece {yon} yone (ekstra) donuluyor...")
    return guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi)


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn, dur_bayragi=None):
    """
    Bir pozisyondan sonrakine gecerken yapilan hareketi uygular.

    GUNCELLEME: "sure" tipi hareketler artik duz_git_sabit_sure_duzeltmeli()
    ile yapiliyor - BNO055 heading geri beslemeli ve yon-duyarli (geri
    giderken duzeltme isareti ters cevriliyor), boylece 5->6'daki
    "yamulan geri gidis" sorunu cozuldu.
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
        olay_fn(f"  Gecis: {gecis['sure_s']}s duz gidiliyor ({yon_sure}, hiz carpani "
                f"{hiz_carpani:.3f}, heading duzeltmeli)...")
        duz_git_sabit_sure_duzeltmeli(bridge, pwm_a, pwm_b, gecis["sure_s"],
                                        yon=yon_sure, hiz_carpani=hiz_carpani,
                                        dur_bayragi=dur_bayragi)

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
    Sabit-aci + otomatik-sweep rotasi - 8 atis pozisyonlu surum
    (kirmizidan 6, yesilden 2).

    skor_dinleyici verilirse: her pozisyonda break-beam sensorlerinden
    ilk basarili gecisten sonra en fazla 2 saniye daha beklenip sweep
    otomatik durur ve GUI'deki skor p["puan"] degerine gore artar.

    dur_bayragi verilirse (threading.Event): set edildiginde tum
    hareketler en kisa surede durdurulur (esc.kapat() HER ZAMAN calisir).

    ESC KURALI: Robot 1. pozisyona ULASANA kadar ESC KAPALI (0) tutulur.

    durum_fn: GUI icin yapilandirilmis asama bildirimi (onceki surumle ayni).
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
        # 8 ATIS POZISYONU (kirmizidan 6, yesilden 2)
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
                    # Gecis oncesi donus modu:
                    #   baseline_atla=True     -> HICBIR SEY geri alinmaz
                    #   sadece_sweep_geri=True -> SADECE sweep geri alinir
                    #                             (robot ILK HEDEF ACIDA kalir,
                    #                             ilk_aci GERI ALINMAZ; gecisin
                    #                             ekstra_donus'u bu hedef acinin
                    #                             UZERINE eklenir)
                    #   (ikisi de yoksa)       -> tam baseline (sweep + ilk_aci geri)
                    if GECISLER[i].get("baseline_atla"):
                        olay_fn(f"  {p['etiket']}: baseline donusu ATLANIYOR - "
                                f"sweep geri alinmadan, mevcut yonde dogrudan "
                                f"gecis hareketine geciliyor...")
                    elif GECISLER[i].get("sadece_sweep_geri"):
                        if sweep_toplam > 0:
                            olay_fn(f"  {p['etiket']}: SADECE sweep geri aliniyor "
                                    f"({sweep_toplam:.1f} derece "
                                    f"{ters_yon(p['sweep_yon'])}) - robot ilk "
                                    f"hedef acida kalacak...")
                            if not guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]),
                                                  bridge, pwm_a, pwm_b,
                                                  dur_bayragi=dur_bayragi):
                                _durdurma_kontrol_et(dur_bayragi)
                                raise RuntimeError(
                                    f"{p['etiket']}: sweep geri alma donusu basarisiz oldu")
                        else:
                            olay_fn(f"  {p['etiket']}: sweep yapilmamis, robot "
                                    f"zaten ilk hedef acida - donus gerekmiyor.")
                    else:
                        baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn,
                                     dur_bayragi=dur_bayragi)
                    gecis_uygula(bridge, pwm_a, pwm_b, GECISLER[i], olay_fn,
                                 dur_bayragi=dur_bayragi)
                # son pozisyonda (8. atis) hicbir geri donus/hareket yok
        except RuntimeError as e:
            olay_fn(f"HATA: {e}")
            _durum_bildir("hata", etiket=str(e))
            return False

        olay_fn("\nOzel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(8 atis pozisyonu: kirmizidan 6, yesilden 2).")
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
