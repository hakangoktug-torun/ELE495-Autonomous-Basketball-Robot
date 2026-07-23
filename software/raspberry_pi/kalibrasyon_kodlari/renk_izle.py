"""
ELE495 - Canli Renk (RGB/C) Izleyici
TCS34725'ten gelen R,G,B,C degerlerini canli gosterir. Renk sensorunun
gercekten veri gonderip gondermedigini dogrulamak icin kullan - robotu
farkli renkli yuzeylerin uzerine koyup degerlerin degisip degismedigini
gozlemleyebilirsin.

Kullanim:
    python3 renk_izle.py

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

    print("Renk izleme basladi. Robotu farkli renkli yuzeylerin uzerine koyup "
          "degerlerin degisip degismedigini gozlemle.\n")

    try:
        while True:
            r, g, b, c = bridge.get_color()
            if r is not None:
                sys.stdout.write(
                    f"\rR={r:.0f}  G={g:.0f}  B={b:.0f}  C={c:.0f}          "
                )
                sys.stdout.flush()
            else:
                sys.stdout.write("\rRenk verisi henuz gelmedi...          ")
                sys.stdout.flush()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nDurduruldu.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
