"""
ELE495 - Canli Kalibrasyon Izleyici
Sadece kalibrasyon degerlerini hizli (her ~0.4s) sorup, ayni satirda
guncelleyerek gosterir. accel degeri her degistiginde ayri bir satirda
vurgulayarak bildirir - boylece hangi pozisyonun ise yaradigini
aninda anlarsin.

Kullanim:
    python3 kalibrasyon_izle.py

Ctrl+C ile durdurabilirsin.
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge

SERIAL_PORT = "/dev/ttyUSB0"
SORGU_ARALIGI = 0.4  # saniye - ne kadar kucukse o kadar hizli guncellenir


def main():
    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()

    print("BNO055 baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)

    if bridge.is_stale(max_age_sec=1.0):
        print("UYARI: BNO055'ten veri gelmiyor, baglantiyi kontrol et.")
        bridge.stop()
        return

    print("Izleme basladi. Sensoru/robotu istedigin pozisyonda tut, deger degisimini gozle.")
    print("Ctrl+C ile cikabilirsin.\n")

    onceki_sys = onceki_gyro = onceki_accel = onceki_mag = None

    try:
        while True:
            bridge.request_calibration_status()
            time.sleep(0.15)  # cevabin islenmesini bekle

            cal = bridge.get_calibration()
            sys_v = cal["sys"] if cal["sys"] is not None else 0
            gyro_v = cal["gyro"] if cal["gyro"] is not None else 0
            accel_v = cal["accel"] if cal["accel"] is not None else 0
            mag_v = cal["mag"] if cal["mag"] is not None else 0

            # Herhangi bir deger degistiyse ayri bir satirda vurgula
            if onceki_accel is not None and accel_v != onceki_accel:
                yon = "ARTTI" if accel_v > onceki_accel else "AZALDI"
                print(f"\n>>> ACCEL DEGISTI: {onceki_accel} -> {accel_v}  ({yon}) <<<\n")
            if onceki_gyro is not None and gyro_v != onceki_gyro:
                yon = "ARTTI" if gyro_v > onceki_gyro else "AZALDI"
                print(f"\n>>> GYRO DEGISTI: {onceki_gyro} -> {gyro_v}  ({yon}) <<<\n")
            if onceki_mag is not None and mag_v != onceki_mag:
                yon = "ARTTI" if mag_v > onceki_mag else "AZALDI"
                print(f"\n>>> MAG DEGISTI: {onceki_mag} -> {mag_v}  ({yon}) <<<\n")
            if onceki_sys is not None and sys_v != onceki_sys:
                yon = "ARTTI" if sys_v > onceki_sys else "AZALDI"
                print(f"\n>>> SYS DEGISTI: {onceki_sys} -> {sys_v}  ({yon}) <<<\n")

            onceki_sys, onceki_gyro, onceki_accel, onceki_mag = sys_v, gyro_v, accel_v, mag_v

            # Ayni satirda surekli guncellenen canli gosterge
            sys.stdout.write(
                f"\rsys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v}   "
                f"(0=kotu, 3=mukemmel)          "
            )
            sys.stdout.flush()

            time.sleep(max(0.0, SORGU_ARALIGI - 0.15))

    except KeyboardInterrupt:
        print("\n\nDurduruldu.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
