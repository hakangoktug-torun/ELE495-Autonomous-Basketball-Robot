"""
ELE495 - Firlaticisiz Atis Simulasyonu (Kare rota + renkli bolge tespiti)
Atici henuz monte edilmedi, ama monte edilmis GIBI davranip atis icin bir
sure bekleyerek simule ediyoruz.

Bu dosya IKI SEKILDE kullanilabilir:
  1) Komut satirindan direkt calistirilabilir (eskisi gibi, input() ile
     senden yon/aci sorar):
         python3 firlaticisiz_atis_simulasyon_test.py
  2) Baska bir kod (orn. Flask GUI) tarafindan IMPORT EDILIP
     calistir_atis_dongusu() fonksiyonu cagrilabilir - bu durumda
     input() KULLANILMAZ, bunun yerine caller'in verdigi callback
     fonksiyonlari (aci_getir_fn, olay_fn, puan_fn) kullanilir. Boylece
     ayni mantik hem komut satirinda hem web arayuzunde birebir calisir.

Her 4 kenarda tekrarlanan ADIMLAR:
  1) Onundeki cisme YAKLASMA_ESIGI_CM kalana kadar duz git
  2) Renk sensorunden bulunulan bolgeyi oku ve bildir
     (kirmizi = 3 puanlik bolge, yesil = 2 puanlik bolge)
  3) Atis acisini al (CLI'da sorulur, GUI'de web'den beklenir)
  4) O aciyla don
  5) Atis icin bir sure bekle (simulasyon - gercek atici yok)
  6) Ayni aciyla TERS yone donup baslangic yonune geri don
  7) 90 derece sola don (bir sonraki kenara gecis)

Bu dongu 4 kez tekrarlanir - robot boylece bir KARE cizer.

Bu dosyayi ayni klasore koy: software/raspberry_pi/kalibrasyon_kodlari/
(donus_kapali_dongu.py, robot_bridge.py ve test_surus.py ile ayni yerde olmali)
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge
from donus_kapali_dongu import motorlari_ayarla, motorlari_durdur, SERIAL_PORT
from test_surus import guvenli_donus, ileri_git_engel_bulunca

TOPLAM_KENAR_SAYISI = 4
YAKLASMA_ESIGI_CM = 15.0
ATIS_BEKLEME_SURESI = 2.0  # saniye - gercek atici olsaydi atis icin gereken sure (simulasyon)


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


def calistir_atis_dongusu(bridge, pwm_a, pwm_b,
                            kenar_sayisi=TOPLAM_KENAR_SAYISI,
                            yaklasma_esigi_cm=YAKLASMA_ESIGI_CM,
                            atis_bekleme_suresi=ATIS_BEKLEME_SURESI,
                            aci_getir_fn=None,
                            olay_fn=print,
                            puan_fn=None):
    """
    Atis simulasyonu dongusunu calistirir - CLI ve GUI arasinda PAYLASILAN
    tek mantik burasi.

    aci_getir_fn(kenar_no, bolge_aciklama, bolge_puan) -> (yon, aci)
        Cagrildiginda BLOKE OLABILIR (CLI'da input() bekler, GUI'de web'den
        cevap gelene kadar bekler). None verilirse hata firlatilir.
    olay_fn(mesaj)
        Her onemli adimda cagrilir (log/tarihce icin). Varsayilan: print.
    puan_fn(puan)
        Bir bolgeye ulasildiginda o bolgenin puanini bildirir. None ise
        yoksayilir (sadece olay_fn ile bildirilir).

    Donus deger: True (kare tamamlandi) / False (bir adimda basarisiz oldu)
    """
    if aci_getir_fn is None:
        raise ValueError("aci_getir_fn saglanmali (CLI icin cli_aci_getir kullan)")

    for kenar_no in range(1, kenar_sayisi + 1):
        olay_fn(f"--- {kenar_no}. kenar: hedefe yaklasiliyor ---")
        bulundu = ileri_git_engel_bulunca(bridge, pwm_a, pwm_b, esik_cm=yaklasma_esigi_cm)
        if not bulundu:
            olay_fn(f"{kenar_no}. kenarda hedef bulunamadi, durduruluyor.")
            return False

        # ---- Bolge/renk tespiti ----
        r, g, b, c = bridge.get_color()
        aciklama, puan = bolge_belirle(r, g, b, c)
        olay_fn(f"Bolge tespiti: {aciklama}  (R={r}, G={g}, B={b}, C={c})")
        if puan_fn is not None:
            puan_fn(puan)

        # ---- Atis acisini al ----
        yon, aci = aci_getir_fn(kenar_no, aciklama, puan)
        ters_yon = "sol" if yon == "sag" else "sag"

        # ---- Atis acisina don ----
        olay_fn(f"{aci} derece {yon} yone donuluyor (atis pozisyonu)...")
        if not guvenli_donus(aci, yon, bridge, pwm_a, pwm_b):
            olay_fn(f"{kenar_no}. kenarda atis donusu basarisiz oldu, durduruluyor.")
            return False

        # ---- Atis simulasyonu (gercek atici yok, sadece bekleme) ----
        olay_fn(f"Atici simule ediliyor - {atis_bekleme_suresi}s bekleniyor "
                f"(gercek atici olsaydi burada atis yapardi)...")
        time.sleep(atis_bekleme_suresi)

        # ---- Baslangic yonune geri don ----
        olay_fn(f"{aci} derece {ters_yon} yone donup baslangic yonune donuluyor...")
        if not guvenli_donus(aci, ters_yon, bridge, pwm_a, pwm_b):
            olay_fn(f"{kenar_no}. kenarda geri donus basarisiz oldu, durduruluyor.")
            return False

        # ---- 90 derece sola don (bir sonraki kenara gecis) ----
        olay_fn(f"{kenar_no}. donus: 90 derece sola (sonraki kenara gecis)")
        if not guvenli_donus(90, "sol", bridge, pwm_a, pwm_b):
            olay_fn(f"{kenar_no}. kenarda 90 derecelik gecis donusu basarisiz oldu, durduruluyor.")
            return False

    olay_fn(f"Atis simulasyonu tamamlandi (kare tamamlandi - toplam {kenar_sayisi} kenar).")
    return True


# ---------------------------------------------------------------------------
# CLI modu - dogrudan "python3 firlaticisiz_atis_simulasyon_test.py" ile
# calistirildiginda kullanilir.
# ---------------------------------------------------------------------------

def cli_aci_getir(kenar_no, bolge_aciklama, bolge_puan):
    """CLI modunda: kullanicidan input() ile yon/aci ister."""
    print(f"\n=== {kenar_no}. KENAR - ATIS ACISI ===")
    while True:
        yon = input("Atis icin donus yonu (sol/sag): ").strip().lower()
        if yon in ("sol", "sag"):
            break
        print("Gecersiz - 'sol' ya da 'sag' yaz.")

    while True:
        aci_str = input("Atis icin donus acisi (derece, orn: 30): ").strip()
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

    pwm_a, pwm_b = motorlari_ayarla()

    toplam_puan = [0]

    def cli_puan_ekle(puan):
        toplam_puan[0] += puan

    try:
        calistir_atis_dongusu(
            bridge, pwm_a, pwm_b,
            aci_getir_fn=cli_aci_getir,
            olay_fn=print,
            puan_fn=cli_puan_ekle,
        )
        print(f"\nToplam tahmini puan: {toplam_puan[0]}")

    finally:
        motorlari_durdur(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        bridge.stop()


if __name__ == "__main__":
    main()
