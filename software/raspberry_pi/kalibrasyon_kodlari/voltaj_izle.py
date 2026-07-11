"""
ELE495 - Canli Vcc (Gerilim) Izleyici
Arduino'nun 5V hattinin gercek gerilimini surekli gosterir - motorlar
calisirken bu deger dusuyorsa, güc kaynagi (pil/powerbank) yetersiz kaliyor
demektir. Bunu ayri bir terminalde acik birakip, baska bir terminalde
donus/surus testleri calistirarak canli karsilastirma yapabilirsin.

Kullanim:
    python3 voltaj_izle.py

Ctrl+C ile durdurabilirsin.
"""

import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from robot_bridge import RobotBridge

SERIAL_PORT = "/dev/ttyUSB0"
DUSUK_GERILIM_ESIGI = 4300  # mV - bunun altinda uyari ver


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

    print("Vcc izleme basladi. Motorlari baska bir terminalden calistirip "
          "gerilimin ne kadar dustugunu gozlemleyebilirsin.")
    print(f"Dusuk gerilim esigi: {DUSUK_GERILIM_ESIGI}mV\n")

    en_dusuk_gorulen = None

    try:
        while True:
            vcc = bridge.get_vcc()
            if vcc is not None:
                if en_dusuk_gorulen is None or vcc < en_dusuk_gorulen:
                    en_dusuk_gorulen = vcc

                uyari = ""
                if vcc < DUSUK_GERILIM_ESIGI:
                    uyari = "  <-- DUSUK GERILIM UYARISI"

                sys.stdout.write(
                    f"\rVcc = {vcc:.0f} mV   (en dusuk gorulen: "
                    f"{en_dusuk_gorulen:.0f} mV){uyari}          "
                )
                sys.stdout.flush()

            time.sleep(0.1)

    except KeyboardInterrupt:
        if en_dusuk_gorulen:
            print(f"\n\nDurduruldu. Test boyunca en dusuk gorulen gerilim: "
                  f"{en_dusuk_gorulen:.0f} mV")
        else:
            print("\n\nDurduruldu.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
