"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

GUNCELLEME (BU SURUM - kullanici istegiyle, "2->3 ultrasonik 8cm esikli durus"):
  2->3 GECISI ARTIK ZAMANLI/TEK-OLCUMLU DEGIL, ULTRASONIK TABANLI: eskiden
  bu gecis "mesafe" tipindeydi - SADECE bir kez anlik mesafe olculup
  (sadece "yeterli aciklik var mi" diye ikili kontrol icin), gercek
  ilerleme ise ZAMANA guvenerek sabit 10cm gidiliyordu. Artik hareket=
  "engel" moduna cevrildi: ileri_git_engel_bulunca() ile SUREKLI GERCEK
  ultrasonik mesafe olculerek, onundeki hedefe esik_cm=8.0 (yani 8cm)
  kalinca robot 3. konuma ULASMIS SAYILIP otomatik duruyor. Bu sayede
  robot HER SEFERINDE ayni fiziksel konuma (onundeki nesneden 8cm) gelir -
  eskiden zamanli sabit 10cm ilerleme, robotun 2. pozisyondan 3.
  pozisyona TAM OLARAK ne kadar uzakta baslayacagina gore degisken bir
  son nokta veriyordu; artik mesafe olcumu bu degiskenligi ortadan kaldirir.

  ONEMLI: gecis_uygula()'daki "engel" isleyicisi esik_cm'e OTOMATIK
  +25cm'lik bir "telafi payi" (esik_telafi_cm, varsayilan 25.0) ekliyor -
  bu gecis icin esik_telafi_cm=0.0 ACIKCA belirtildi, yoksa robot
  gercekte cok daha ileride (33cm'de) duracakti. Fiziksel testte robot
  8cm'den belirgin uzak/yakin duruyorsa esik_cm'i 1-2cm adimlarla ayarla
  (bkz. 3->4 ve 6->7 gecislerindeki benzer "telafi" ogrenimleri).

  Ayrica bu gecis artik test_surus.py'deki YENI "hayalet mesafe sicramasi"
  filtresinden de otomatik faydalanir (ileri_git_engel_bulunca ortak
  fonksiyon oldugu icin) - ani/fiziksel olarak imkansiz mesafe dususleri
  artik bir sonraki ornekle dogrulanmadan kabul edilmiyor.

  (1-8. pozisyon acilari/ESC hizlari ve diger tum gecisler AYNEN KORUNDU -
  bu surumde SADECE 2->3 gecisinin hareket tipi/esik degeri degisti.)

ROTA (8 pozisyon - guncel acilar):
  1. atis (Kirmizi, 3p): 89 derece SOL, ESC %11.3.
  1->2 : sweep geri, hedef acidan 89 derece SAG, 10cm ileri.
  2. atis (Kirmizi, 3p): 102 derece SOL, ESC %11.3.
  2->3 : sweep geri, hedef acidan 102 derece SAG, sonra ULTRASONIK ile
         onundeki hedefe 8cm kalana kadar ILERI (YENI - eskiden mesafe
         kontrollu tek-olcum + sabit 10cm zamanli hareketti).
  3. atis (Kirmizi, 3p): 114 derece SOL, ESC %11.3.
  3->4 : sweep geri, hedef acidan 24 derece SAG, sonra ULTRASONIK ile
         onundeki hedefe 49cm kalana kadar ILERI (hiz_carpani=0.6 ile
         yavaslatilmis), yesil dogrulamali.
  4. atis (Yesil, 2p): 36 derece SOL, ESC %9.6.
  4->5 : sweep geri, hedef acidan 54 derece SOL (36+54=90 net), 10cm ileri.
  5. atis (Yesil, 2p): 84 derece SAG, ESC %10.
  5->6 : sweep geri, hedef aciya 6 derece SAG eklenir (84+6=90 net),
         duraksamadan 1.2s DUZ GERI (heading duzeltmeli). Kirmizi dogrulanir.
  6. atis (Kirmizi, 3p): 4 derece SOL, ESC %11.3.
  6->7 : sweep geri, hedef acidan 92 derece SAG, sonra ULTRASONIK ile
         duvara 8cm kalana kadar ILERI.
  7. atis (Kirmizi, 3p): 118 derece SOL, ESC %11.2.
  7->8 : sweep geri, hedef acidan 62 derece SOL, 0.8s ILERI.
  8. atis (Kirmizi, 3p): 95 derece SAG, ESC %11.2 - SON POZISYON, rota biter.

ONEMLI TASARIM NOTLARI:
  - GECIS MODU "sadece_sweep_geri": bu bayragi tasiyan gecislerde sweep
    acilari geri alinir ama pozisyonun ilk_aci donusu GERI ALINMAZ -
    robot ilk HEDEF ACIDA kalir ve gecisin ekstra_donus'u bu hedef
    acinin UZERINE eklenir. ("baseline_atla" bayragi da hala
    destekleniyor - o hicbir seyi geri almaz.)
  - TEK PARCA DONUSLER: tum bolme mantiklari kaldirilmis durumda - her
    donus tek hamlede.

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
    sonlandirilmasi icin kullanilan sinyal istisnasi.
    """
    pass


def _durdurma_kontrol_et(dur_bayragi):
    if dur_bayragi is not None and dur_bayragi.is_set():
        raise TestDurduruldu()


# ---------- Duzeltmeli duz gitme ----------
DUZ_KAZANC_P = 0.4
DUZ_MAKS_DUZELTME = 10.0


def duz_git_sabit_sure_duzeltmeli(bridge, pwm_a, pwm_b, sure_s, yon="ileri",
                                    hiz_carpani=1.0, dur_bayragi=None):
    """
    BNO055 heading geri beslemeli, sabit sureli DUZ gitme - hem ILERI hem
    GERI yonde dogru calisir. yon="geri" iken duzeltme isareti ters
    cevrilir (geri giderken ayni isaret hatayi buyutur, yamulmaya sebep
    olurdu).
    """
    hedef_heading = bridge.get_heading()

    if yon == "geri":
        geri_yon_ayarla()
        isaret = -1.0
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


def _buyuk_aci_donus_uygula(aci, yon, bridge, pwm_a, pwm_b, olay_fn, dur_bayragi=None):
    """Bir donusu TEK PARCA uygular. aci <= 0 ise donus TAMAMEN ATLANIR."""
    if aci <= 0:
        return True
    return guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi)


# ---------- Pozisyon tanimlari ----------
POZISYON_1 = dict(ilk_yon="sol", ilk_aci=89.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="1. atis", puan=3)
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=102.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="2. atis", puan=3)
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=114.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="3. atis", puan=3)
POZISYON_4 = dict(ilk_yon="sol", ilk_aci=36.0, esc_hiz=9.6,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="4. atis", puan=2)
POZISYON_5 = dict(ilk_yon="sag", ilk_aci=84.0, esc_hiz=9.4,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="5. atis", puan=2)
POZISYON_6 = dict(ilk_yon="sol", ilk_aci=8.0, esc_hiz=11.3,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="6. atis", puan=3)
POZISYON_7 = dict(ilk_yon="sol", ilk_aci=116.0, esc_hiz=11,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="7. atis", puan=3)

POZISYON_8 = dict(ilk_yon="sag", ilk_aci=95.0, esc_hiz=11.2,
                   sweep_yon="sag", sweep_adim=2.0, sweep_bekleme=10.0,
                   maks_sweep=3, etiket="8. atis", puan=3)

POZISYONLAR = [POZISYON_1, POZISYON_2, POZISYON_3, POZISYON_4, POZISYON_5,
               POZISYON_6, POZISYON_7, POZISYON_8]

# ---------- Pozisyonlar arasi gecisler ----------
GECISLER = [
    dict(ekstra_donus=("sag", 89.0), hareket="mesafe",
         sadece_sweep_geri=True),                                                      # 1 -> 2
    dict(ekstra_donus=("sag", 102.0), hareket="engel", esik_cm=8.0, esik_telafi_cm=0.0,
         sadece_sweep_geri=True),                                                      # 2 -> 3 (GUNCELLENDI: kullanici istegiyle - eskiden "mesafe" tipiydi (tek anlik olcum + sabit 10cm zamanli ileri). Artik ULTRASONIK ile onundeki hedefe 8cm KALANA kadar surekli olculerek ilerliyor, 3. konuma bu mesafede ULASMIS SAYILIYOR - boylece robot HER SEFERINDE ayni fiziksel konuma gelir. esik_telafi_cm=0.0 ACIKCA belirtildi - yoksa gecis_uygula varsayilan +25cm telafi ekleyip robotu gercekte 33cm'de durdururdu.)
    dict(ekstra_donus=("sag", 24.0), hareket="engel", esik_cm=49.0, esik_telafi_cm=0.0,
         renk_dogrula=True, hiz_carpani=0.6, sadece_sweep_geri=True),                  # 3 -> 4
    dict(ekstra_donus=("sol", 54.0), hareket="mesafe",
         sadece_sweep_geri=True),                                                      # 4 -> 5
    dict(ekstra_donus=("sag", 6.0), hareket="sure", sure_s=1.2, yon="geri",
         renk_dogrula=True, hedef_bolge="KIRMIZI", sadece_sweep_geri=True),            # 5 -> 6
    dict(ekstra_donus=("sag", 98.0), hareket="engel", esik_cm=8.0, esik_telafi_cm=0.0,
         hiz_carpani=0.5, sadece_sweep_geri=True),                                     # 6 -> 7
    dict(ekstra_donus=("sol", 64.0), hareket="sure", sure_s=0.8,
         sadece_sweep_geri=True),                                                      # 7 -> 8
]


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=10.0, skor_dinleyici=None, dur_bayragi=None,
                esc=None, esc_hiz_kontrolcusu=None):
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
                        f"(ya da 2. basarili gecis geldi) - pozisyon "
                        f"tamamlandi, sweep durduruluyor.")
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
                olay_fn(f"  [{etiket}] UYARI: sweep donusu basarisiz oldu, "
                        f"sweep durduruluyor.")
                break
            sweep_toplam += sweep_adim
            guncel_hiz = esc_hiz_kontrolcusu.get() if esc_hiz_kontrolcusu is not None else esc_hiz
            esc.hiz_ayarla(guncel_hiz)

        return sweep_toplam

    finally:
        if skor_dinleyici is not None:
            skor_dinleyici.saymayi_durdur()
        esc.hiz_ayarla(0)
        olay_fn(f"  [{etiket}] ESC durduruldu (0) - pozisyon tamamlandi.")


def pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn,
                       skor_dinleyici=None, puan=0, dur_bayragi=None,
                       esc_hiz_kontrolcusu=None):
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
    olay_fn(f"  Gecis: {aci} derece {yon} yone (ekstra) donuluyor...")
    return guvenli_donus(aci, yon, bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi)


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn, dur_bayragi=None):
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
    esc = EscKontrol(pin=esc_pin)
    esc.baslat()
    esc.hiz_ayarla(0)

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
        olay_fn("1. adim: 30 derece saga donuluyor...")
        if not guvenli_donus(30, "sag", bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("1. adim basarisiz oldu, durduruluyor.")
            return False

        olay_fn("2. adim: 0.5 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.5, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        olay_fn("3. adim: 60 derece saga donuluyor...")
        if not guvenli_donus(60, "sag", bridge, pwm_a, pwm_b, dur_bayragi=dur_bayragi):
            _durdurma_kontrol_et(dur_bayragi)
            olay_fn("3. adim basarisiz oldu, durduruluyor.")
            return False

        olay_fn("4. adim: 0.3 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 0.3, dur_bayragi=dur_bayragi)
        _durdurma_kontrol_et(dur_bayragi)

        try:
            for i, p in enumerate(POZISYONLAR):
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
