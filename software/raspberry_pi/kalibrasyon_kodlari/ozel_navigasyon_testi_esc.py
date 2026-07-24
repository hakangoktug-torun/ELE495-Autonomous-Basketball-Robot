"""
ELE495 - Ozel Navigasyon Test Rotasi (4 kullanici girdili - 4 atis pozisyonu)
test_surus.py'nin kare rotasindan FARKLI, ozel bir hareket dizisi dener.
Robot 4 farkli pozisyonda durur (ilk 2'si 3 puanlik bolgede, son 2'si 2
puanlik bolgede) ve her pozisyonda senden bir atis acisi/yonu ister.

GUNCELLEME (acil durdur destegi): ileri_git_sabit_sure() artik opsiyonel
bir dur_bayragi (threading.Event) parametresi aliyor - set edilirse hareket
en kisa surede durdurulup fonksiyondan cikilir.

Bu dosya IKI SEKILDE kullanilabilir (Flask GUI ile
ayni mantik):
  1) Komut satirindan direkt calistirilabilir (input() ile sorar):
         python3 ozel_navigasyon_testi_esc.py
  2) Baska bir kod (Flask GUI) tarafindan import edilip
     calistir_ozel_rota() cagrilabilir - input() yerine caller'in verdigi
     callback fonksiyonlari (aci_getir_fn, olay_fn) kullanilir.

ADIMLAR:
  1) Saga 30 derece don
  2) 0.5 saniye duz git
  3) Saga 60 derece don
  4) 0.3 saniye duz git
  5) Renk sensorunden bolge tespiti + Kullanicidan yon/aci sor (1. atis - 3 puanlik bolge)
  6) O aciyla don
  7) Ayni aciyla TERS yone donup eski haline geri gel
  8) Ultrasonik mesafe oku - onde 10cm'den fazla acikliksa, 10cm ileri git
  9) Renk sensorunden bolge tespiti + Kullanicidan yon/aci sor (2. atis - 3 puanlik bolge)
  10) O aciyla don
  11) Ayni aciyla TERS yone donup eski haline geri gel
  12) 10 cm daha ilerle
  13) 90 derece sola don
  14) 1 saniye duz git
  15) Renk sensorunden bolge tespiti + Kullanicidan yon/aci sor (3. atis - 2 puanlik bolge)
  16) O aciyla don
  17) Ayni aciyla TERS yone donup eski haline geri gel
  18) 90 derece sola don
  19) 10 cm ilerle
  20) Renk sensorunden bolge tespiti + Kullanicidan yon/aci sor (4. atis - 2 puanlik bolge)
  21) O aciyla don
  22) Bitir

NOT: Bolge tespiti KESIN degil - basit bir RGB oran karsilastirmasi (bkz.
bolge_belirle()). Sahadaki gercek kirmizi/yesil renklerin R,G,B oranlarini
renk_izle.py ile olcup gerekirse esikleri ince ayar yapman gerekebilir.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve test_surus.py ile ayni yerde olmali)
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import (
    motorlari_ayarla, motorlari_durdur, aci_farki, SERIAL_PORT
)
from test_surus import (
    guvenli_donus, ileri_yon_ayarla, geri_yon_ayarla, ileri_git_sabit_mesafe,
    SOL_HIZ, SAG_HIZ, DUZELTME_KAZANCI, DUZELTME_KAZANCI_I, MAKS_DUZELTME,
    MAKS_INTEGRAL,
)
from atici_esc_kontrol_pigpio_2 import EscKontrol

MIN_ACIKLIK_CM = 10.0     # onde bu kadardan fazla aciklik varsa ilerle
ILERLEME_MESAFESI_CM = 10.0  # ne kadar ilerlenecek
ESC_PIN = 17  # atici ESC'sinin sinyal pini


def _durdurma_istendi_mi(dur_bayragi):
    """Ortak yardimci - dur_bayragi verilmis ve set edilmisse True doner."""
    return dur_bayragi is not None and dur_bayragi.is_set()


def bolge_belirle(r, g, b, c):
    """
    Ham RGB degerlerine bakarak kabaca hangi renk bolgesinde oldugunu tahmin
    eder. Bu, KESIN bir renk taniyici DEGIL - basit bir oransal karsilastirma.
    Gercek sahada test ederken esikleri (0.4 gibi) senin gordugun gercek
    degerlere gore ince ayar yapman gerekebilir (renk_izle.py ile olcup
    kirmizi/yesil zeminde ne gibi R,G,B oranlari cikiyor gozlemleyebilirsin).

    Donus deger: (aciklama_metni, puan)
    """
    if r is None or g is None or b is None:
        return "bilinmiyor (veri yok)", 0

    toplam = r + g + b
    if toplam <= 0:
        return "bilinmiyor (gecersiz okuma)", 0

    r_oran = r / toplam
    g_oran = g / toplam
    b_oran = b / toplam

    if r_oran > g_oran and r_oran > b_oran and r_oran > 0.40:
        return "KIRMIZI (3 puanlik bolge)", 3
    elif g_oran > r_oran and g_oran > b_oran and g_oran > 0.40:
        return "YESIL (2 puanlik bolge)", 2
    else:
        return f"taninmayan bolge (R oran={r_oran:.2f}, G oran={g_oran:.2f}, B oran={b_oran:.2f})", 0


def bolge_bildir(bridge, olay_fn):
    """Renk sensorunden anlik okuma alip bolgeyi tahmin eder ve olay_fn ile bildirir."""
    r, g, b, c = bridge.get_color()
    aciklama, puan = bolge_belirle(r, g, b, c)
    olay_fn(f"Bolge tespiti: {aciklama} (R={r}, G={g}, B={b}, C={c})")
    return aciklama, puan


def ileri_git_sabit_sure(bridge, pwm_a, pwm_b, sure_saniye, dur_bayragi=None,
                          hiz_carpani=1.0, yon="ileri"):
    """
    Belirtilen sure boyunca duz ILERI ya da GERI gider (mesafe sensoru
    KULLANILMAZ, sadece zaman). BNO055 heading feedback ile saga/sola
    kaymayi anlik olarak duzeltir.

    yon: (YENI) "ileri" (varsayilan) ya da "geri". "geri" verilirse
    geri_yon_ayarla() ile motor pinleri TERS cevrilip robot GERIYE gider.
    NOT: Heading-duzeltme mantigi (hangi tekerin hizlandirilip
    yavaslatilacagi) ILERI yon icin kalibre edilmis - GERI giderken de
    ayni mantik uygulaniyor. Kisa (orn. <0.5s) geri hareketlerde bu
    yeterince dogru calisir; cok uzun geri hareketler icin ince ayar
    gerekebilir.

    hiz_carpani: taban hizi (SOL_HIZ/SAG_HIZ) bu katsayiyla carpar -
    SADECE bu cagriya ozel, global SOL_HIZ/SAG_HIZ SABIT kalir. 1.0 =
    degisiklik yok (varsayilan).

    dur_bayragi: set edilirse hareket en kisa surede durdurulup
    fonksiyondan cikilir.
    """
    if _durdurma_istendi_mi(dur_bayragi):
        print("DURDURMA sinyali - sabit sureli hareket baslamadan iptal edildi.")
        return

    if yon == "geri":
        geri_yon_ayarla()
    else:
        ileri_yon_ayarla()

    hedef_heading = bridge.get_heading()
    temel_sol, temel_sag = SOL_HIZ * hiz_carpani, SAG_HIZ * hiz_carpani
    pwm_a.ChangeDutyCycle(temel_sol)
    pwm_b.ChangeDutyCycle(temel_sag)

    integral = 0.0
    son_zaman = time.time()
    baslangic = time.time()

    while time.time() - baslangic < sure_saniye:
        if _durdurma_istendi_mi(dur_bayragi):
            print("DURDURMA sinyali - sabit sureli hareket iptal ediliyor.")
            break

        simdiki_heading = bridge.get_heading()
        simdi = time.time()
        dt = simdi - son_zaman
        son_zaman = simdi

        if simdiki_heading is not None and hedef_heading is not None:
            hata = aci_farki(hedef_heading, simdiki_heading)

            integral += hata * dt
            integral = max(-MAKS_INTEGRAL, min(MAKS_INTEGRAL, integral))

            duzeltme = DUZELTME_KAZANCI * hata + DUZELTME_KAZANCI_I * integral
            duzeltme = max(-MAKS_DUZELTME, min(MAKS_DUZELTME, duzeltme))
        else:
            duzeltme = 0.0

        sol_duty = max(0, min(100, temel_sol - duzeltme))
        sag_duty = max(0, min(100, temel_sag + duzeltme))
        pwm_a.ChangeDutyCycle(sol_duty)
        pwm_b.ChangeDutyCycle(sag_duty)

        time.sleep(0.03)

    motorlari_durdur(pwm_a, pwm_b)


def calistir_ozel_rota(bridge, pwm_a, pwm_b, aci_getir_fn, olay_fn=print, esc_atis_fn=None):
    """
    Yukaridaki hareket dizisini calistirir - CLI ve GUI arasinda PAYLASILAN
    tek mantik burasi.

    NOT: Bu fonksiyon artik GUI (app.py) tarafindan kullanilmiyor - onun
    yerine ozel_navigasyon_testi_esc_sweep_2.py'deki calistir_ozel_rota_sweep()
    kullaniliyor. Bu fonksiyon sadece CLI/legacy kullanim icin korunuyor,
    dur_bayragi entegrasyonu YAPILMADI.

    aci_getir_fn(baglam) -> (yon, aci)
        baglam: {"asama": 1-4, "mesaj": "..."} - hangi asamada oldugumuzu
        ve kullaniciya gosterilecek aciklamayi tasir. Cagrildiginda BLOKE
        OLABILIR (CLI'da input() bekler, GUI'de web'den cevap gelene kadar
        bekler).
    olay_fn(mesaj)
        Her onemli adimda cagrilir (log/tarihce icin). Varsayilan: print.
    esc_atis_fn(baglam) -> None
        Robot HER 4 atis pozisyonuna da ulastiginda cagrilir - kullanici
        ESC hizini ayarlayip atisi deneyip 'devam' diyene kadar bloke olur.
        None verilirse (CLI'da varsayilan), bu dosyanin icindeki basit CLI
        tabanli ESC kontrolu kullanilir (atici_esc_kontrol_pigpio.EscKontrol).

    Donus deger: True (tum adimlar basarili) / False (bir adimda basarisiz oldu)
    """
    if esc_atis_fn is None:
        esc_atis_fn = _cli_esc_atis
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

    # ---- 5) Bolge tespiti + kullanicidan yon/aci sor (1. atis - 3 puanlik bolge) ----
    bolge_bildir(bridge, olay_fn)
    olay_fn("5. adim: ilk atis icin yon/aci bekleniyor...")
    yon1, aci1 = aci_getir_fn({
        "asama": 1,
        "mesaj": "1. atis (3 puanlik bolge) - istedigin yon ve aciyi gir",
    })

    # ---- 6) O aciyla don ----
    olay_fn(f"6. adim: {aci1} derece {yon1} yone donuluyor...")
    if not guvenli_donus(aci1, yon1, bridge, pwm_a, pwm_b):
        olay_fn("6. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 6.5) ATIS - 1. pozisyon (3 puanlik bolge) ----
    olay_fn("1. atis pozisyonuna ulasildi - ESC hizi ayarlanip atis bekleniyor...")
    esc_atis_fn({"asama": 1, "mesaj": "1. atis (3 puanlik bolge) - ESC hizini ayarla, atisi dene, sonra devam et"})
    olay_fn("1. atis tamamlandi, devam ediliyor.")

    # ---- 7) Ayni aciyla TERS yone donup eski haline geri gel ----
    ters_yon1 = "sol" if yon1 == "sag" else "sag"
    olay_fn(f"7. adim: {aci1} derece {ters_yon1} yone donup eski haline geri donuluyor...")
    if not guvenli_donus(aci1, ters_yon1, bridge, pwm_a, pwm_b):
        olay_fn("7. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 8) Ultrasonik mesafe oku - onde 10cm'den fazla aciklik varsa 10cm ilerle ----
    mesafe = bridge.get_distance()
    olay_fn(f"8. adim: ultrasonik mesafe olculdu: {mesafe} cm")
    if mesafe is not None and mesafe > MIN_ACIKLIK_CM:
        olay_fn(f"8. adim: onde yeterli aciklik var, {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
        ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)
    else:
        olay_fn("8. adim: onde yeterli aciklik yok (10cm ve altinda), ilerleme atlaniyor.")

    # ---- 9) Bolge tespiti + kullanicidan yon/aci sor (2. atis - 3 puanlik bolge) ----
    bolge_bildir(bridge, olay_fn)
    olay_fn("9. adim: ikinci atis icin yon/aci bekleniyor...")
    yon2, aci2 = aci_getir_fn({
        "asama": 2,
        "mesaj": "2. atis (3 puanlik bolge) - istedigin yon ve aciyi gir",
    })

    # ---- 10) O aciyla don ----
    olay_fn(f"10. adim: {aci2} derece {yon2} yone donuluyor...")
    if not guvenli_donus(aci2, yon2, bridge, pwm_a, pwm_b):
        olay_fn("10. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 10.5) ATIS - 2. pozisyon (3 puanlik bolge) ----
    olay_fn("2. atis pozisyonuna ulasildi - ESC hizi ayarlanip atis bekleniyor...")
    esc_atis_fn({"asama": 2, "mesaj": "2. atis (3 puanlik bolge) - ESC hizini ayarla, atisi dene, sonra devam et"})
    olay_fn("2. atis tamamlandi, devam ediliyor.")

    # ---- 11) Ayni aciyla TERS yone donup eski haline geri gel ----
    ters_yon2 = "sol" if yon2 == "sag" else "sag"
    olay_fn(f"11. adim: {aci2} derece {ters_yon2} yone donup eski haline geri donuluyor...")
    if not guvenli_donus(aci2, ters_yon2, bridge, pwm_a, pwm_b):
        olay_fn("11. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 12) 10 cm daha ilerle ----
    olay_fn(f"12. adim: {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
    ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)

    # ---- 13) 90 derece sola don ----
    olay_fn("13. adim: 90 derece sola donuluyor...")
    if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
        olay_fn("13. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 14) 1 saniye duz git ----
    olay_fn("14. adim: 1 saniye duz gidiliyor...")
    ileri_git_sabit_sure(bridge, pwm_a, pwm_b, 1.0)

    # ---- 15) Bolge tespiti + kullanicidan yon/aci sor (3. atis - 2 puanlik bolge) ----
    bolge_bildir(bridge, olay_fn)
    olay_fn("15. adim: ucuncu atis icin yon/aci bekleniyor...")
    yon3, aci3 = aci_getir_fn({
        "asama": 3,
        "mesaj": "3. atis (2 puanlik bolge) - istedigin yon ve aciyi gir",
    })

    # ---- 16) O aciyla don ----
    olay_fn(f"16. adim: {aci3} derece {yon3} yone donuluyor...")
    if not guvenli_donus(aci3, yon3, bridge, pwm_a, pwm_b):
        olay_fn("16. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 16.5) ATIS - 3. pozisyon (2 puanlik bolge) ----
    olay_fn("3. atis pozisyonuna ulasildi - ESC hizi ayarlanip atis bekleniyor...")
    esc_atis_fn({"asama": 3, "mesaj": "3. atis (2 puanlik bolge) - ESC hizini ayarla, atisi dene, sonra devam et"})
    olay_fn("3. atis tamamlandi, devam ediliyor.")

    # ---- 17) Ayni aciyla TERS yone donup eski haline geri gel ----
    ters_yon3 = "sol" if yon3 == "sag" else "sag"
    olay_fn(f"17. adim: {aci3} derece {ters_yon3} yone donup eski haline geri donuluyor...")
    if not guvenli_donus(aci3, ters_yon3, bridge, pwm_a, pwm_b):
        olay_fn("17. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 18) 90 derece sola don ----
    olay_fn("18. adim: 90 derece sola donuluyor...")
    if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
        olay_fn("18. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 19) 10 cm ilerle ----
    olay_fn(f"19. adim: {ILERLEME_MESAFESI_CM}cm ilerleniyor...")
    ileri_git_sabit_mesafe(pwm_a, pwm_b, ILERLEME_MESAFESI_CM, bridge=bridge)

    # ---- 20) Bolge tespiti + kullanicidan yon/aci sor (4. atis - 2 puanlik bolge) ----
    bolge_bildir(bridge, olay_fn)
    olay_fn("20. adim: dorduncu atis icin yon/aci bekleniyor...")
    yon4, aci4 = aci_getir_fn({
        "asama": 4,
        "mesaj": "4. atis (2 puanlik bolge) - istedigin yon ve aciyi gir",
    })

    # ---- 21) O aciyla don ----
    olay_fn(f"21. adim: {aci4} derece {yon4} yone donuluyor...")
    if not guvenli_donus(aci4, yon4, bridge, pwm_a, pwm_b):
        olay_fn("21. adim basarisiz oldu, durduruluyor.")
        return False

    # ---- 21.5) ATIS - 4. pozisyon (2 puanlik bolge) ----
    olay_fn("4. atis pozisyonuna ulasildi - ESC hizi ayarlanip atis bekleniyor...")
    esc_atis_fn({"asama": 4, "mesaj": "4. atis (2 puanlik bolge) - ESC hizini ayarla, atisi dene, sonra devam et"})
    olay_fn("4. atis tamamlandi.")

    olay_fn("Ozel navigasyon test rotasi tamamlandi (4 atis pozisyonu).")
    return True


def sensor_kontrolu(bridge, sure_saniye=2.0):
    """
    Test baslamadan once TUM sensorleri (BNO055 heading, ultrasonik mesafe,
    IR, renk, Vcc) birkac saniye boyunca orneklyip her birinin gecerli veri
    verip vermedigini kontrol eder. Ozellikle ultrasonik sensorun ara sira
    verdigi gecersiz (-1) okumalari yakalamak icin TEK bir aninlik okumaya
    degil, birden fazla ornege bakilir.

    Donus deger: tespit edilen sorunlarin metin listesi (bos liste = sorun yok)
    """
    print("\n=== SENSOR ON KONTROLU ===")
    print(f"{sure_saniye}s boyunca tum sensorler ornekleniyor, lutfen bekle...")

    ornekler = {"heading": [], "distance": [], "ir1": [], "ir2": [], "renk": [], "vcc": []}
    baslangic = time.time()
    while time.time() - baslangic < sure_saniye:
        veri = bridge.get_all()
        ornekler["heading"].append(veri["heading"])
        ornekler["distance"].append(veri["distance"])
        ornekler["ir1"].append(veri["ir1"])
        ornekler["ir2"].append(veri["ir2"])
        ornekler["renk"].append((veri["r"], veri["g"], veri["b"], veri["c"]))
        ornekler["vcc"].append(veri["vcc_mv"])
        time.sleep(0.05)

    sorunlar = []

    # ---- BNO055 (heading) ----
    heading_gecerli = [h for h in ornekler["heading"] if h is not None]
    if not heading_gecerli:
        print("  [HATA] BNO055 (heading): hic veri alinamadi.")
        sorunlar.append("BNO055 (heading): veri alinamadi.")
    else:
        print(f"  [OK] BNO055 (heading): {len(heading_gecerli)} gecerli okuma, "
              f"son deger={heading_gecerli[-1]:.1f} derece")

    # ---- Ultrasonik (mesafe) ----
    toplam_mesafe_ornegi = len(ornekler["distance"])
    mesafe_gecerli = [d for d in ornekler["distance"] if d is not None and d > 0]
    gecersiz_sayisi = toplam_mesafe_ornegi - len(mesafe_gecerli)

    if not mesafe_gecerli:
        print("  [HATA] Ultrasonik (mesafe): tum okumalar gecersiz (-1/None).")
        sorunlar.append("Ultrasonik (mesafe): tum okumalar gecersiz.")
    else:
        gecersiz_oran = 100 * gecersiz_sayisi / toplam_mesafe_ornegi if toplam_mesafe_ornegi else 0
        if gecersiz_sayisi > 0:
            print(f"  [UYARI] Ultrasonik (mesafe): {gecersiz_sayisi}/{toplam_mesafe_ornegi} "
                  f"gecersiz okuma (%{gecersiz_oran:.0f}), son gecerli deger={mesafe_gecerli[-1]:.1f}cm")
            if gecersiz_oran > 20:
                sorunlar.append(f"Ultrasonik (mesafe): gecersiz okuma orani yuksek (%{gecersiz_oran:.0f}).")
        else:
            print(f"  [OK] Ultrasonik (mesafe): tum okumalar gecerli, son deger={mesafe_gecerli[-1]:.1f}cm")

    # ---- IR sensorleri ----
    ir1_gecerli = [v for v in ornekler["ir1"] if v is not None]
    ir2_gecerli = [v for v in ornekler["ir2"] if v is not None]
    if not ir1_gecerli or not ir2_gecerli:
        print("  [HATA] IR sensorleri: veri alinamadi.")
        sorunlar.append("IR sensorleri: veri alinamadi.")
    else:
        print(f"  [OK] IR sensorleri: IR1={ir1_gecerli[-1]:.0f} IR2={ir2_gecerli[-1]:.0f}")

    # ---- Renk sensoru ----
    renk_gecerli = [r for r in ornekler["renk"] if r[0] is not None]
    if not renk_gecerli:
        print("  [HATA] Renk sensoru (TCS34725): veri alinamadi.")
        sorunlar.append("Renk sensoru: veri alinamadi.")
    else:
        son = renk_gecerli[-1]
        print(f"  [OK] Renk sensoru: R={son[0]:.0f} G={son[1]:.0f} B={son[2]:.0f} C={son[3]:.0f}")

    # ---- Vcc (guc) ----
    vcc_gecerli = [v for v in ornekler["vcc"] if v is not None]
    if not vcc_gecerli:
        print("  [HATA] Vcc (guc) okumasi: veri alinamadi.")
        sorunlar.append("Vcc: veri alinamadi.")
    else:
        son_vcc = vcc_gecerli[-1]
        if son_vcc < 4300:
            print(f"  [UYARI] Vcc dusuk: {son_vcc:.0f}mV (esik: 4300mV) - "
                  f"guc kaynagi yetersiz kalabilir.")
            sorunlar.append(f"Vcc dusuk ({son_vcc:.0f}mV).")
        else:
            print(f"  [OK] Vcc: {son_vcc:.0f}mV")

    print()
    if sorunlar:
        print("Tespit edilen olasi sorunlar:")
        for s in sorunlar:
            print(f"  - {s}")
    else:
        print("Hicbir sorun tespit edilmedi.")
    print()

    return sorunlar


# ---------------------------------------------------------------------------
# CLI modu
# ---------------------------------------------------------------------------

def _cli_esc_atis(baglam):
    """
    CLI modunda varsayilan ESC kontrolu - EscKontrol'u dogrudan kullanir,
    kullanicidan interaktif olarak hiz yuzdesi ister, 'devam' yazana kadar
    bekler.
    """
    print(f"\n=== {baglam['mesaj']} ===")
    esc = EscKontrol(pin=ESC_PIN)
    esc.baslat()
    try:
        while True:
            girdi = input("ESC hizi (0-100), atisi bitirip devam etmek icin 'devam' yaz: ").strip().lower()
            if girdi == "devam":
                break
            try:
                yuzde = float(girdi)
                uygulanan = esc.hiz_ayarla(yuzde)
                print(f"Hiz %{uygulanan:.2f} olarak ayarlandi.")
            except ValueError:
                print("Gecerli bir sayi ya da 'devam' yaz.")
    finally:
        esc.kapat()


def cli_aci_getir(baglam):
    print(f"\n=== {baglam['mesaj']} ===")
    while True:
        yon = input("Donus yonu (sol/sag): ").strip().lower()
        if yon in ("sol", "sag"):
            break
        print("Gecersiz - 'sol' ya da 'sag' yaz.")

    while True:
        aci_str = input("Donus acisi (derece, orn: 30): ").strip()
        try:
            aci = float(aci_str)
            if aci > 0:
                break
            print("Aci pozitif bir sayi olmali.")
        except ValueError:
            print("Gecerli bir sayi gir.")

    return yon, aci


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

    # ---- Baslamadan once tum sensorleri kontrol et ----
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
        calistir_ozel_rota(bridge, pwm_a, pwm_b, aci_getir_fn=cli_aci_getir, olay_fn=print)

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
