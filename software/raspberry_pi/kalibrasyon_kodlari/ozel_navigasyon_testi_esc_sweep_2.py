"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

ozel_navigasyon_testi_esc.py'nin ayni iskeletini kullanir (1-4. adimlar
ayni: 30 derece sag -> 0.5s duz -> 60 derece sag -> 0.3s duz), ama HER
atis pozisyonunda kullanicidan yon/aci SORMAK yerine SABIT bir baslangic
acisiyla doner, ESC'yi calistirir, sonra kullanicidan komut gelene kadar
3'er derecelik adimlarla ACISAL TARAMA (sweep) yaparak ayni bolgeye
farkli acilardan atmayi dener.

AMAC: Robotun donuslerde birkac derecelik sapma yasayabilmesi yuzunden
("tam 90 dönemiyor" sorunu) TEK bir sabit aciya guvenmek yerine, o
acinin etrafindaki birkac komsu aciyi da otomatik deneyerek potansiyel
sapmanin sonucunu (top cemberden gecmemesi) telafi etmeye calisir.

POZISYONLAR (6 atis: kirmizidan 3, yesilden 3):
  1. atis (Kirmizi): 87 derece SOL, ESC %12.4, sweep SAGA, maks 3 tekrar.
  2. atis (Kirmizi, YENI): 90 derece SOL, ESC %12.4, sweep SAGA, maks 3.
  3. atis (Kirmizi): 90 derece SOL, ESC %12.4, sweep SAGA, maks 3.
  4. atis (Yesil): 10 derece SOL, ESC %11, sweep SAGA, maks 3.
  5. atis (Yesil): 100 derece SAG, ESC %11, sweep SOLA (yon TERS!), maks 2.
  6. atis (Yesil, YENI): 100 derece SAG, ESC %11, sweep SAGA, maks 3.
     Bu pozisyondan sonra robot GERI DONMEZ, kod biter.

GECISLER (pozisyonlar arasi hareket, HER ZAMAN once baseline'a - yani
konuma varilan giris acisina - tam geri donulerek yapilir):
  1->2, 2->3: sadece 10cm ileri (donus yok, ayni kirmizi bolge icinde).
  3->4 (KIRMIZI -> YESIL): 90 derece sola donup 0.4 saniye duz gidilir
       (onceden 1 saniyeydi, guncellendi).
  4->5: ekstra 90 derece sola donup 10cm ileri gidilir.
  5->6 (YENI): ekstra 90 derece sola donup 10cm ileri gidilir - ayni
       4->5 gecisiyle ayni sablon (VARSAYIM - kullanicidan "90 sola
       donup 10cm duz git" denildi, baseline'a once donulup donulmedigi
       acikca belirtilmedi; tutarlilik icin diger gecislerdeki gibi
       once baseline'a donuluyor kabul edildi).

SURE BUTCESI (5 dakikalik demo siniri icin kaba tahmin): her sweep
adimi 20s bekleme + donus suresi demek. 1/2/3/4. pozisyonlarda maks 3
tekrar (~63s'ye kadar her biri), 5. pozisyonda maks 2 (~42s), 6.
pozisyonda maks 3 (~63s). Kullanicinin 'sonraki'/'atla' ile beklemeyi
kisaltmasi ya da 'devam' ile pozisyonu erken bitirmesi bu sureyi
onemli olcude azaltir - demoda 5 dakika sinirini asmamak icin bu
komutlari aktif kullanmak onerilir.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(ozel_navigasyon_testi_esc.py, donus_kapali_dongu.py, robot_bridge.py,
test_surus.py ve atici_esc_kontrol_pigpio_2.py ile ayni yerde olmali)
"""

import sys
import os
import select
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
from test_surus import guvenli_donus, ileri_git_sabit_mesafe
from atici_esc_kontrol_pigpio_2 import EscKontrol

from ozel_navigasyon_testi_esc import (
    bolge_bildir, ileri_git_sabit_sure, sensor_kontrolu,
    MIN_ACIKLIK_CM, ILERLEME_MESAFESI_CM, ESC_PIN,
)

# ---------- Pozisyon tanimlari (deneysel - kolay ayarlanabilir) ----------
POZISYON_1 = dict(ilk_yon="sol", ilk_aci=87.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3, etiket="1. atis")
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=90.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3, etiket="2. atis")
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=100.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3, etiket="3. atis")
POZISYON_4 = dict(ilk_yon="sol", ilk_aci=10.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3, etiket="4. atis")
POZISYON_5 = dict(ilk_yon="sag", ilk_aci=100.0, esc_hiz=11.0,
                   sweep_yon="sol", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=2, etiket="5. atis")
POZISYON_6 = dict(ilk_yon="sag", ilk_aci=100.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
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
#                        (ultrasonik kontrolu YOK - kirmizi->yesil gecisi
#                        icin, cok kisa bir hareket oldugu icin)
GECISLER = [
    dict(ekstra_donus=None, hareket="mesafe"),                                  # 1 -> 2
    dict(ekstra_donus=None, hareket="mesafe"),                                  # 2 -> 3
    dict(ekstra_donus=("sol", 90.0), hareket="sure", sure_s=0.4),               # 3 -> 4 (kirmizi->yesil)
    dict(ekstra_donus=("sol", 90.0), hareket="mesafe"),                        # 4 -> 5
    dict(ekstra_donus=("sag", 90.0), hareket="mesafe"),                        # 5 -> 6 (YENI, varsayim) başta soldu
]


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=10.0):
    """
    'sure' saniye boyunca STDIN'den bir komut gelip gelmedigini, TAM
    'sure' kadar bloke olmadan (select() ile non-blocking) kontrol eder.

    Iki farkli komut ayirt edilir:
      'devam'   -> kullanici bu POZISYONU tamamen bitirmek istiyor
      'sonraki' (ya da 'atla')
                -> kullanici sadece bu sweep adimini erken bitirip HEMEN
                   bir sonraki sweep acisina gecmek istiyor - pozisyon
                   bitmiyor, kalan bekleme suresi atlaniyor

    Donus: "devam"    -> kullanici pozisyonu bitirmek istedi
           "sonraki"  -> kullanici bir sonraki sweep acisina hemen gecmek istedi
           None       -> sure doldu, hicbir komut gelmedi (normal akis, sweep'e devam)
    """
    bitis = time.time() + sure
    while True:
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
                    sweep_adim, sweep_bekleme, maks_sweep, olay_fn, etiket):
    """
    ESC'yi esc_hiz'e ayarlayip atisa birakir, sonra su iki komuttan biri
    gelene ya da (varsa) maks_sweep sinirina ulasilana kadar her
    sweep_bekleme saniyede bir sweep_adim derece sweep_yon yonune donup
    ESC'yi tekrar ayarlar:

      'devam'   -> bu pozisyon tamamen biter (sweep durur, fonksiyon doner)
      'sonraki' (ya da 'atla')
                -> kalan bekleme suresi atlanip HEMEN bir sonraki sweep
                   acisina gecilir (pozisyon bitmez, sweep sayaci normal
                   isler - sadece bekleme kisaltilmis olur)

    Hicbir komut gelmezse (sure dolarsa) normal akista bir sonraki sweep
    adimina geciliyor - yani 'sonraki' ile zaman asiminin sonucu AYNI,
    tek fark 'sonraki' 20s beklemeden hemen tetikliyor olmasi.

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
        olay_fn(f"  [{etiket}] {sweep_bekleme:.0f}s bekleniyor - 'devam' "
                f"yazarsan pozisyon biter, 'sonraki' (ya da 'atla') yazarsan "
                f"beklemeden hemen bir sonraki aciya gecilir...")
        komut = girdi_bekle(sweep_bekleme)
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
        if not guvenli_donus(sweep_adim, sweep_yon, bridge, pwm_a, pwm_b):
            olay_fn(f"  [{etiket}] UYARI: sweep donusu basarisiz oldu, "
                    f"sweep durduruluyor.")
            break
        sweep_toplam += sweep_adim
        # ESC'yi donus sonrasi tekrar ayarla - bazi ESC/regulator'ler
        # sinyal kesintisinde failsafe'e girip durabiliyor, guvenlik icin.
        esc.hiz_ayarla(esc_hiz)

    return sweep_toplam


def pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn):
    """
    Bir atis pozisyonunda: ilk donusu yapar, ESC'yi ayarlayip sweep
    atisini calistirir. Sweep'i GERI ALMAZ - bu, baseline_don() ile
    ayri bir adimda yapilir (boylece son pozisyonda hic geri donmeden
    biraktirilabilir).

    Donus: sweep_toplam (derece, sweep_yon yonunde toplam donulen miktar)
    """
    olay_fn(f"\n=== {p['etiket']}: {p['ilk_aci']} derece {p['ilk_yon']} "
            f"yone donuluyor ===")
    if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b):
        raise RuntimeError(f"{p['etiket']}: ilk donus basarisiz oldu")

    return sweep_atis_yap(
        bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
        p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
        olay_fn, p["etiket"],
    )


def baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn):
    """
    Bir pozisyonun sweep'ini geri alip (varsa), sonra ilk donusun
    TERSINE donerek pozisyona GIRIS acisina (baseline, 0 derece relatif)
    tam olarak geri doner.
    """
    if sweep_toplam > 0:
        olay_fn(f"  {p['etiket']}: sweep geri aliniyor ({sweep_toplam:.1f} "
                f"derece {ters_yon(p['sweep_yon'])})...")
        if not guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b):
            raise RuntimeError(f"{p['etiket']}: sweep geri alma donusu basarisiz oldu")
    olay_fn(f"  {p['etiket']}: {p['ilk_aci']} derece {ters_yon(p['ilk_yon'])} "
            f"yone donup baslangica (0 derece) geri donuluyor...")
    if not guvenli_donus(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b):
        raise RuntimeError(f"{p['etiket']}: baseline donusu basarisiz oldu")


def gecis_uygula(bridge, pwm_a, pwm_b, gecis, olay_fn):
    """
    Bir pozisyondan sonrakine gecerken yapilan hareketi uygular
    (baseline_don() cagrildiktan SONRA cagrilmali). GECISLER listesindeki
    bir gecis sozlugunu alir - bkz. dosya basindaki GECISLER aciklamasi.
    """
    if gecis.get("ekstra_donus"):
        yon, aci = gecis["ekstra_donus"]
        olay_fn(f"  Gecis: {aci} derece {yon} yone (ekstra) donuluyor...")
        if not guvenli_donus(aci, yon, bridge, pwm_a, pwm_b):
            raise RuntimeError("Gecis donusu basarisiz oldu")

    if gecis["hareket"] == "mesafe":
        mesafe = bridge.get_distance()
        olay_fn(f"  Gecis: ultrasonik mesafe olculdu: {mesafe} cm")
        if mesafe is not None and mesafe > MIN_ACIKLIK_CM:
            olay_fn(f"  Gecis: onde yeterli aciklik var, "
                    f"{ILERLEME_MESAFESI_CM}cm ilerleniyor...")
            ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)
        else:
            olay_fn("  Gecis: onde yeterli aciklik yok, ilerleme atlaniyor.")
    elif gecis["hareket"] == "sure":
        olay_fn(f"  Gecis: {gecis['sure_s']}s duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, gecis["sure_s"])


def calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print, esc_pin=ESC_PIN):
    """
    calistir_ozel_rota()'nin kullanicidan aci sorma kismini kaldirip
    yerine sabit-aci + otomatik-sweep mantigi koyan, 6 atis pozisyonlu
    (kirmizidan 3, yesilden 3) versiyonu.
    """
    esc = EscKontrol(pin=esc_pin)
    esc.baslat()

    try:
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

        # =====================================================
        # 6 ATIS POZISYONU (kirmizidan 3, yesilden 3)
        # =====================================================
        try:
            for i, p in enumerate(POZISYONLAR):
                bolge_bildir(bridge, olay_fn)
                sweep_toplam = pozisyon_calistir(bridge, pwm_a, pwm_b, esc, p, olay_fn)

                son_pozisyon = (i == len(POZISYONLAR) - 1)
                if not son_pozisyon:
                    baseline_don(bridge, pwm_a, pwm_b, p, sweep_toplam, olay_fn)
                    gecis_uygula(bridge, pwm_a, pwm_b, GECISLER[i], olay_fn)
                # son pozisyonda (6. atis) hicbir geri donus/hareket yok
        except RuntimeError as e:
            olay_fn(f"HATA: {e}")
            return False

        olay_fn("\nOzel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(6 atis pozisyonu: kirmizidan 3, yesilden 3).")
        return True

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
