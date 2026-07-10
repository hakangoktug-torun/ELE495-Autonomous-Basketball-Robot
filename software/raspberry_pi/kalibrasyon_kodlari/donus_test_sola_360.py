"""
ELE495 - Tek basina donus testi: 360 derece SOLA (tam tur)
Robotun uzerine bir ok/isaret koyup, donus oncesi ve sonrasi ayni yone
bakip bakmadigini gozle kontrol edebilirsin.

Bu dosyayi kalibrasyon_kodlari/ klasorune koy, donus_kapali_dongu.py ile ayni yerde olmali.

Kullanim:
    python3 donus_test_sola_360.py
"""

import time
from donus_kapali_dongu import donus_yap, RobotBridge, SERIAL_PORT

if __name__ == "__main__":
    bridge = RobotBridge(port=SERIAL_PORT)
    bridge.start()

    print("BNO055 baglantisi bekleniyor...")
    for _ in range(50):
        if not bridge.is_stale(max_age_sec=1.0):
            break
        time.sleep(0.1)

    # EEPROM'da kayitli kalibrasyonun, KAYIT ANINDAKI kalitesini goster
    bridge.request_saved_calibration_info()
    time.sleep(0.3)
    kayitli = bridge.get_saved_calibration_info()
    print(f"Kayitli kalibrasyon bilgisi (kaydedildigi andaki degerler) -> "
          f"sys={kayitli['sys']} gyro={kayitli['gyro']} "
          f"accel={kayitli['accel']} mag={kayitli['mag']}\n")

    donus_yap(360, yon="sol", bridge=bridge)
    bridge.stop()
