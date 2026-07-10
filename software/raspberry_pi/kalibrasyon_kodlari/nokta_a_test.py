"""
ELE495 - Nokta A'da Potaya Yonelme Testi
Gercek saha olmadan, sadece GEOMETRIYI test eder: robot A noktasindaymis
gibi varsayip, potaya bakmak icin kac derece ve hangi yone donmesi
gerektigini hesaplar, sonra bu donusu gercekten uygular.

VARSAYIMLAR (KONTROL ET):
  1. Nokta A: (21, 15)  - onceki haritalamadan
  2. Pota konumu: (100, 40) - TAHMIN, kesin degilse asagidan duzelt
  3. Robot, A'ya varirken +x yonune (potaya dogru olan genel eksene) bakiyor
     kabul ediliyor - yani su anki "ileri" yonu = potaya dogru genel yon.
  4. Koordinat sistemi: x sahanin uzun ekseni (0=baslangic duvari, 120=pota
     duvari), y sahanin kisa ekseni (0-80).

Kullanim:
    python3 nokta_a_test.py
"""

import sys
import os
import math
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from donus_kapali_dongu import donus_yap, RobotBridge, SERIAL_PORT

# ---------- Konum bilgileri (GEREKIRSE DUZELT) ----------
NOKTA_A = (21, 15)
POTA_KONUM = (100, 40)  # TAHMIN - kesin koordinati biliyorsan burayi guncelle


def gerekli_donusu_hesapla(baslangic_nokta, hedef_nokta):
    """
    baslangic_nokta'dan hedef_nokta'ya bakmak icin, robotun +x yonune
    (mevcut 'ileri' yonune) gore kac derece ve hangi yone (sol/sag)
    donmesi gerektigini hesaplar.

    Koordinat sistemi varsayimi: x=sahanin uzun ekseni, y=kisa eksen,
    y ASAGI dogru artiyor (ekrandaki gibi). Robot +x yonune bakiyor.
    Bu durumda +y yonu robotun SAGINDA kalir (dogu'ya bakan biri icin
    guney sagda kalir gibi) - yani dy>0 ise SAGA, dy<0 ise SOLA donulmeli.

    Donus deger: (aci_derece, yon) - yon 'sol' ya da 'sag'
    """
    dx = hedef_nokta[0] - baslangic_nokta[0]
    dy = hedef_nokta[1] - baslangic_nokta[1]

    aci_derece = math.degrees(math.atan2(abs(dy), dx))
    yon = "sag" if dy > 0 else "sol" if dy < 0 else None

    return aci_derece, yon, dx, dy


def main():
    aci, yon, dx, dy = gerekli_donusu_hesapla(NOKTA_A, POTA_KONUM)

    print("=== NOKTA A - POTAYA YONELME HESABI ===")
    print(f"Nokta A: {NOKTA_A}")
    print(f"Pota konumu (tahmini): {POTA_KONUM}")
    print(f"dx={dx:.1f}, dy={dy:.1f}")
    print(f"Gerekli donus: {aci:.1f} derece, yon: {yon}\n")

    if yon is None:
        print("Pota tam karsida (dy=0), donus gerekmiyor.")
        return

    onay = input(f"Bu hesapla devam edip robotu {aci:.1f} derece {yon} yone "
                 f"dondurelim mi? (yes/no): ").strip().lower()
    if onay != "yes":
        print("Iptal edildi. NOKTA_A / POTA_KONUM degerlerini duzeltip tekrar dene.")
        return

    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()

    print("\nBNO055 baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)

    if bridge.is_stale(max_age_sec=1.0):
        print("UYARI: Sensor veri akisi yok, baglantiyi kontrol et.")
        bridge.stop()
        return

    print(f"\nRobot {aci:.1f} derece {yon} yone donduruluyor (potaya yonelme)...\n")
    sonuc = donus_yap(aci, yon=yon, bridge=bridge)

    print(f"\nDonus tamamlandi. Robotun simdi potaya baktigini gozle kontrol et "
          f"(A noktasinda oldugunu varsayarak).")

    bridge.stop()


if __name__ == "__main__":
    main()
