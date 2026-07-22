"""
ELE495 - Ozel Navigasyon Test Rotasi (SWEEP versiyonu - kullanici girdisiz aci)

ozel_navigasyon_testi_esc.py'nin ayni iskeletini kullanir (1-4. adimlar
ayni: 30 derece sag -> 0.5s duz -> 60 derece sag -> 0.3s duz), ama HER
atis pozisyonunda kullanicidan yon/aci SORMAK yerine SABIT bir baslangic
acisiyla doner, ESC'yi calistirir, sonra kullanicidan 'devam' gelene
kadar 3'er derecelik adimlarla ACISAL TARAMA (sweep) yaparak ayni bolgeye
farkli acilardan atmayi dener.

AMAC: Robotun donuslerde birkac derecelik sapma yasayabilmesi yuzunden
("tam 90 dönemiyor" sorunu) TEK bir sabit aciya guvenmek yerine, o
acinin etrafindaki birkac komsu aciyi da otomatik deneyerek potansiyel
sapmanin sonucunu (top cemberden gecmemesi) telafi etmeye calisir.

POZISYONLAR (kullanicinin verdigi spesifikasyona gore):
  1. atis (3 puanlik bolge): 87 derece SOL, ESC %12.4, sweep SAGA
     3'er derece, 20s araliklarla, EN FAZLA 3 TEKRAR (demo suresi
     5 dakikayla sinirli oldugu icin sweep artik sinirsiz degil).
  2. atis: 90 derece SOL, ESC %12.4, sweep SAGA 3'er derece, EN FAZLA
     3 TEKRAR.
  3. atis (2 puanlik bolge): 10 derece SOL, ESC %11, sweep SAGA
     3'er derece, EN FAZLA 3 TEKRAR. 4. konuma gecerken 1. ve 2.
     pozisyonla AYNI mantik: once sweep geri alinir, sonra baslangica
     (0) TAM geri donulur, SONRA oradan EKSTRA olarak 90 derece daha
     sola donulur.
  4. atis (son pozisyon): 100 derece SAG, ESC %11, sweep SOLA (yon
     TERS!) 3'er derece, EN FAZLA 2 TEKRAR. Bu pozisyondan sonra robot
     GERI DONMEZ, kod biter.

SURE BUTCESI (5 dakikalik demo siniri icin kaba tahmin): her sweep
adimi 20s bekleme + donus suresi (~1-2s) demek. 1/2/3. pozisyonlarda
3 tekrara kadar cikarsa (~63s'ye kadar) + 4. pozisyonda 2 tekrara kadar
(~42s'ye kadar) + donuslere/ilerlemelere harcanan sure - toplamda en
kotu senaryoda (hicbir pozisyonda erken 'devam' gelmezse) yaklasik
3.5-4 dakikaya kadar cikabilir. Kullanicinin gerektiginde erken 'devam'
yazmasi bu suреyi kisaltir.

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
                   maks_sweep=3)
POZISYON_2 = dict(ilk_yon="sol", ilk_aci=90.0, esc_hiz=12.4,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3)
POZISYON_3 = dict(ilk_yon="sol", ilk_aci=10.0, esc_hiz=11.0,
                   sweep_yon="sag", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=3)
POZISYON_4 = dict(ilk_yon="sag", ilk_aci=100.0, esc_hiz=11.0,
                   sweep_yon="sol", sweep_adim=3.0, sweep_bekleme=20.0,
                   maks_sweep=2)


def ters_yon(yon):
    return "sol" if yon == "sag" else "sag"


def girdi_bekle(sure=20.0):
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


def calistir_ozel_rota_sweep(bridge, pwm_a, pwm_b, olay_fn=print, esc_pin=ESC_PIN):
    """
    calistir_ozel_rota()'nin kullanicidan aci sorma kismini kaldirip
    yerine sabit-aci + otomatik-sweep mantigi koyan versiyonu.
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
        # POZISYON 1 (3 puanlik bolge)
        # =====================================================
        bolge_bildir(bridge, olay_fn)
        p = POZISYON_1
        olay_fn(f"5. adim: {p['ilk_aci']} derece {p['ilk_yon']} yone "
                f"donuluyor (1. atis)...")
        if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b):
            olay_fn("5. adim basarisiz oldu, durduruluyor.")
            return False

        sweep_toplam = sweep_atis_yap(
            bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
            p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
            olay_fn, "1. atis",
        )

        # Geri donus: once sweep'i geri al (ilk acisina don), sonra ilk
        # acinin TERSINE donup baslangic yonune (0) geri gel.
        if sweep_toplam > 0:
            olay_fn(f"  1. atis: sweep geri aliniyor ({sweep_toplam:.1f} "
                    f"derece {ters_yon(p['sweep_yon'])})...")
            guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b)
        olay_fn(f"7. adim: {p['ilk_aci']} derece {ters_yon(p['ilk_yon'])} "
                f"yone donup eski haline geri donuluyor...")
        if not guvenli_donus(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b):
            olay_fn("7. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 8) Ultrasonik mesafe oku - onde aciklik varsa ilerle ----
        mesafe = bridge.get_distance()
        olay_fn(f"8. adim: ultrasonik mesafe olculdu: {mesafe} cm")
        if mesafe is not None and mesafe > MIN_ACIKLIK_CM:
            olay_fn(f"8. adim: onde yeterli aciklik var, "
                    f"{ILERLEME_MESAFESI_CM}cm ilerleniyor...")
            ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)
        else:
            olay_fn("8. adim: onde yeterli aciklik yok, ilerleme atlaniyor.")

        # =====================================================
        # POZISYON 2
        # =====================================================
        bolge_bildir(bridge, olay_fn)
        p = POZISYON_2
        olay_fn(f"9. adim: {p['ilk_aci']} derece {p['ilk_yon']} yone "
                f"donuluyor (2. atis)...")
        if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b):
            olay_fn("9. adim basarisiz oldu, durduruluyor.")
            return False

        sweep_toplam = sweep_atis_yap(
            bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
            p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
            olay_fn, "2. atis",
        )

        if sweep_toplam > 0:
            olay_fn(f"  2. atis: sweep geri aliniyor ({sweep_toplam:.1f} "
                    f"derece {ters_yon(p['sweep_yon'])})...")
            guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b)
        olay_fn(f"11. adim: {p['ilk_aci']} derece {ters_yon(p['ilk_yon'])} "
                f"yone donup eski haline geri donuluyor...")
        if not guvenli_donus(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b):
            olay_fn("11. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 12) 10 cm daha ilerle ----
        olay_fn(f"12. adim: {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
        ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)

        # ---- 13) 90 derece sola don (2. konumdan 3. konuma gecis) ----
        olay_fn("13. adim: 90 derece sola donuluyor...")
        if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
            olay_fn("13. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 14) 0.4 saniye duz git ----
        olay_fn("14. adim: 0.4 saniye duz gidiliyor...")
        ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 1.0)

        # =====================================================
        # POZISYON 3 (2 puanlik bolge)
        # =====================================================
        bolge_bildir(bridge, olay_fn)
        p = POZISYON_3
        olay_fn(f"15. adim: {p['ilk_aci']} derece {p['ilk_yon']} yone "
                f"donuluyor (3. atis)...")
        if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b):
            olay_fn("15. adim basarisiz oldu, durduruluyor.")
            return False

        sweep_toplam = sweep_atis_yap(
            bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
            p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
            olay_fn, "3. atis",
        )

        # 3. konumdan 4. konuma gecis - 1. ve 2. pozisyonla AYNI mantik:
        # once sweep geri alinir, sonra ilk acinin TERSINE donup
        # baslangic yonune (0) TAM geri donulur, SONRA oradan EKSTRA
        # olarak 90 derece daha sola donulur (3->4 gecis hareketi).
        if sweep_toplam > 0:
            olay_fn(f"  3. atis: sweep geri aliniyor ({sweep_toplam:.1f} "
                    f"derece {ters_yon(p['sweep_yon'])})...")
            guvenli_donus(sweep_toplam, ters_yon(p["sweep_yon"]), bridge, pwm_a, pwm_b)
        olay_fn(f"17. adim: {p['ilk_aci']} derece {ters_yon(p['ilk_yon'])} "
                f"yone donup eski haline geri donuluyor...")
        if not guvenli_donus(p["ilk_aci"], ters_yon(p["ilk_yon"]), bridge, pwm_a, pwm_b):
            olay_fn("17. adim basarisiz oldu, durduruluyor.")
            return False

        olay_fn("18. adim: 4. konuma gecis icin 90 derece daha sola donuluyor...")
        if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
            olay_fn("18. adim basarisiz oldu, durduruluyor.")
            return False

        # ---- 4. konuma ilerle ----
        olay_fn(f"19. adim: {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
        ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)

        # =====================================================
        # POZISYON 4 (son pozisyon - sweep SOLA, en fazla 2 tekrar,
        # bittiginde geri donus YOK - kod biter)
        # =====================================================
        bolge_bildir(bridge, olay_fn)
        p = POZISYON_4
        olay_fn(f"20. adim: {p['ilk_aci']} derece {p['ilk_yon']} yone "
                f"donuluyor (4. atis)...")
        if not guvenli_donus(p["ilk_aci"], p["ilk_yon"], bridge, pwm_a, pwm_b):
            olay_fn("20. adim basarisiz oldu, durduruluyor.")
            return False

        sweep_atis_yap(
            bridge, pwm_a, pwm_b, esc, p["esc_hiz"], p["sweep_yon"],
            p["sweep_adim"], p["sweep_bekleme"], p["maks_sweep"],
            olay_fn, "4. atis",
        )

        olay_fn("Ozel navigasyon test rotasi (sweep versiyonu) tamamlandi "
                "(4 atis pozisyonu).")
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
