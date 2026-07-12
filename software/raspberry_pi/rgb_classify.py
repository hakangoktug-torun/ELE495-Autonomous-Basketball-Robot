#!/usr/bin/env python3
import serial
import time
import json
import os
import sys

PORT = "/dev/ttyUSB0"
BAUD = 115200
CAL_FILE = "rgb_calibration.json"

MIN_CLEAR = 200            # obje yok / cok uzak esigi
DOMINANCE_MARGIN = 1.15    # renk baskinligi orani
ACHROMATIC_SPREAD = 0.12   # normalize R/G/B birbirine bu kadar yakinsa "renksiz" (beyaz/gri/siyah)

def read_serial_values(ser):
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        return tuple(map(int, parts))
    except ValueError:
        return None

def calibrate_white(ser, samples=30):
    print("Beyaz yuzeyi sensore 5-10mm mesafeden tut, kalibrasyon basliyor...")
    time.sleep(2)
    rs, gs, bs = [], [], []
    while len(rs) < samples:
        vals = read_serial_values(ser)
        if vals is None:
            continue
        r, g, b, c = vals
        if c < MIN_CLEAR:
            continue
        rs.append(r); gs.append(g); bs.append(b)

    r_ref = sum(rs) / len(rs)
    g_ref = sum(gs) / len(gs)
    b_ref = sum(bs) / len(bs)

    # En yuksek kanali 1.0 kabul edip digerlerini ona gore olcekliyoruz
    max_ref = max(r_ref, g_ref, b_ref)
    cal = {
        "kr": max_ref / r_ref,
        "kg": max_ref / g_ref,
        "kb": max_ref / b_ref,
    }
    with open(CAL_FILE, "w") as f:
        json.dump(cal, f)

    print(f"Kalibrasyon tamamlandi: {cal}")
    return cal

def load_calibration():
    if os.path.exists(CAL_FILE):
        with open(CAL_FILE) as f:
            return json.load(f)
    return {"kr": 1.0, "kg": 1.0, "kb": 1.0}

def classify_color(r, g, b, c, cal):
    if c < MIN_CLEAR:
        return "BELIRSIZ (obje yok / cok uzak)"

    # Beyaz dengesi uygula
    rn = r * cal["kr"]
    gn = g * cal["kg"]
    bn = b * cal["kb"]

    total = rn + gn + bn
    if total == 0:
        return "BELIRSIZ"

    # Normalize edilmis oranlar (0-1 arasi)
    rp, gp, bp = rn / total, gn / total, bn / total
    spread = max(rp, gp, bp) - min(rp, gp, bp)

    # Akromatik mi? (beyaz / gri / siyah)
    if spread < ACHROMATIC_SPREAD:
        if c > 1500:
            return "BEYAZ"
        elif c > 400:
            return "GRI"
        else:
            return "SIYAH"

    # Renkli: baskin kanali bul
    values = {"KIRMIZI": rn, "YESIL": gn, "MAVI": bn}
    sorted_vals = sorted(values.items(), key=lambda x: x[1], reverse=True)
    en_yuksek_isim, en_yuksek_deger = sorted_vals[0]
    ikinci_isim, ikinci_deger = sorted_vals[1]

    if ikinci_deger == 0 or en_yuksek_deger / ikinci_deger < DOMINANCE_MARGIN:
        return "BELIRSIZ (renk net degil)"

    # Kahverengi ayrimi: R baskin ama toplam parlaklik dusukse (koyu/donuk renk)
    if en_yuksek_isim == "KIRMIZI" and c < 800:
        return "KAHVERENGI"

    return en_yuksek_isim

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    if "--calibrate" in sys.argv or not os.path.exists(CAL_FILE):
        cal = calibrate_white(ser)
    else:
        cal = load_calibration()
        print(f"Mevcut kalibrasyon yuklendi: {cal}")

    print("\nRGB sensor okunuyor... (Ctrl+C ile cik)\n")
    try:
        while True:
            vals = read_serial_values(ser)
            if vals is None:
                continue
            r, g, b, c = vals
            renk = classify_color(r, g, b, c, cal)
            print(f"R={r:5d}  G={g:5d}  B={b:5d}  C={c:5d}   ->   {renk}")
    except KeyboardInterrupt:
        print("\nDurduruldu.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
