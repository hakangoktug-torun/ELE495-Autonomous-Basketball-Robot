"""
ELE495 - Kalibrasyonu Bir Kere Yap ve Kaydet
Bu scripti SADECE BIR KEZ (ya da sensor konumu/ortami degistiginde) calistir.
Seni sys=3'e ulasana kadar kalibrasyon yapmaya yonlendirir, sonra bunu
Arduino'nun EEPROM'una KALICI olarak kaydeder.

Bundan sonra, Arduino her acildiginda (RPi/USB baglantisi kesilip
takilsa bile) bu kayitli kalibrasyonu otomatik yukleyecek - donus_kapali_dongu.py
gibi scriptlerin basinda tekrar tekrar '8 ciz' demene gerek kalmayacak.

NOT: Eger BNO055'in fiziksel konumunu/acisini degistirirsen, ya da
robotu tamamen farkli bir ortama (farkli manyetik alan) tasirsan, bu
kayitli kalibrasyon gecersiz olabilir - o zaman bu scripti tekrar calistir.
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

    print("BNO055 baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)

    if bridge.is_stale(max_age_sec=1.0):
        print("UYARI: Sensor veri akisi yok, baglantiyi kontrol et.")
        bridge.stop()
        return

    print("\n=== KALICI KALIBRASYON ===")
    print("sys=3 (mukemmel) olana kadar bekleniyor. Sensoru/robotu:")
    print("  - GYRO icin: birkac saniye tamamen sabit tut")
    print("  - ACCEL icin: farkli yonlerde (duz, ters, yanlar, egik) sirayla sabit tut")
    print("  - MAG icin: havada '8' cizer gibi cevir\n")

    try:
        while True:
            bridge.request_calibration_status()
            time.sleep(0.3)

            cal = bridge.get_calibration()
            sys_v = cal["sys"] if cal["sys"] is not None else 0
            gyro_v = cal["gyro"] if cal["gyro"] is not None else 0
            accel_v = cal["accel"] if cal["accel"] is not None else 0
            mag_v = cal["mag"] if cal["mag"] is not None else 0

            print(f"sys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v}")

            if sys_v >= 3:
                print("\nsys=3'e ulasildi!")
                cevap = input("Bu kalibrasyonu KALICI olarak kaydetmek istiyor musun? "
                               "(kaydetmek icin 'yes' yaz, daha fazla kalibre etmek "
                               "icin Enter'a bas): ").strip().lower()
                if cevap == "yes":
                    bridge.request_save_calibration()
                    time.sleep(0.3)
                    print("\nKAYDEDILDI. Artik Arduino her acildiginda bu kalibrasyonu "
                          "otomatik yukleyecek.")
                    print("NOT: Sensorun fiziksel konumunu degistirirsen, bu scripti "
                          "tekrar calistirman gerekir.")
                    return

            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\nDurduruldu, kaydetme yapilmadi.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
