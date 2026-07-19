"""
ELE495 - Ultrasonik (HC-SR04) Mesafe Sensoru Test/Izleme Kodu
Mesafe verisini canli gosterir - onune elini/bir cismi koyup uzaklastirarak
sensorun dogru tepki verip vermedigini gozlemleyebilirsin.

Ayrica gecersiz okumalari (-1, yani pulseIn timeout) ve okuma hizini da
sayar - bu, sensorun ne kadar guvenilir calistigini anlamana yardimci olur.

Kullanim:
    python3 ultrasonik_test.py

Ctrl+C ile durdurabilirsin.
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge

SERIAL_PORT = "/dev/ttyUSB0"


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

    print("Mesafe izleme basladi. Sensorun onune bir cisim koyup "
          "uzaklastirarak degerlerin dogru degistigini gozlemle.\n")

    toplam_okuma = 0
    gecersiz_okuma = 0
    en_yakin = None
    en_uzak = None
    son_deger = None

    try:
        while True:
            veri = bridge.get_all()
            mesafe = veri["distance"]
            guncel_zaman_damgasi = veri["last_update"]

            # Sadece GERCEKTEN yeni bir Arduino ornegi geldiginde say (hizli
            # polling ayni bayat degeri birden fazla kez okumasin).
            if mesafe != son_deger:
                son_deger = mesafe
                toplam_okuma += 1

                if mesafe is None or mesafe <= 0:
                    gecersiz_okuma += 1
                    durum = "GECERSIZ (timeout/okunamadi)"
                else:
                    if en_yakin is None or mesafe < en_yakin:
                        en_yakin = mesafe
                    if en_uzak is None or mesafe > en_uzak:
                        en_uzak = mesafe
                    durum = f"{mesafe:.1f} cm"

                gecersiz_oran = (100 * gecersiz_okuma / toplam_okuma) if toplam_okuma else 0
                sys.stdout.write(
                    f"\rMesafe: {durum:<28} | en yakin: "
                    f"{f'{en_yakin:.1f}cm' if en_yakin is not None else '-':<8} | "
                    f"en uzak: {f'{en_uzak:.1f}cm' if en_uzak is not None else '-':<8} | "
                    f"gecersiz: {gecersiz_okuma}/{toplam_okuma} (%{gecersiz_oran:.0f})   "
                )
                sys.stdout.flush()

            time.sleep(0.02)

    except KeyboardInterrupt:
        print(f"\n\nDurduruldu. Toplam {toplam_okuma} okuma, "
              f"{gecersiz_okuma} gecersiz (%{100 * gecersiz_okuma / toplam_okuma if toplam_okuma else 0:.0f}).")
        if en_yakin is not None:
            print(f"En yakin olculen mesafe: {en_yakin:.1f} cm")
        if en_uzak is not None:
            print(f"En uzak olculen mesafe: {en_uzak:.1f} cm")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
