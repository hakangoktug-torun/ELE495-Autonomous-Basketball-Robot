#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

# Bu iki esigi kendi sensorunle kalibre etmen lazim (asagida nasil oldugu anlatiliyor)
MIN_CLEAR = 200      # Bunun altinda "obje yok / cok uzak" say
DOMINANCE_MARGIN = 1.15   # En yuksek kanal, ikinciyi en az %15 gecmeli

def classify_color(r, g, b, c):
    if c < MIN_CLEAR:
        return "BELIRSIZ (obje yok / cok uzak)"

    values = {"KIRMIZI": r, "YESIL": g, "MAVI": b}
    sorted_vals = sorted(values.items(), key=lambda x: x[1], reverse=True)

    en_yuksek_isim, en_yuksek_deger = sorted_vals[0]
    ikinci_isim, ikinci_deger = sorted_vals[1]

    if ikinci_deger == 0:
        return en_yuksek_isim

    if en_yuksek_deger / ikinci_deger >= DOMINANCE_MARGIN:
        return en_yuksek_isim
    else:
        return "BELIRSIZ (renk net degil)"

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print("RGB sensor okunuyor... (Ctrl+C ile cik)\n")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 4:
                continue

            try:
                r, g, b, c = map(int, parts)
            except ValueError:
                continue

            renk = classify_color(r, g, b, c)
            print(f"R={r:5d}  G={g:5d}  B={b:5d}  C={c:5d}   ->   {renk}")

    except KeyboardInterrupt:
        print("\nDurduruldu.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
