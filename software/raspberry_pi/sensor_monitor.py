import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

print("Sensor Monitor Baslatiliyor...")
print("Durdurmak icin Ctrl+C")
print("-" * 40)

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            parts = line.split(',')
            if len(parts) == 3:
                ir1  = 'ENGEL' if parts[0] == '0' else 'temiz'
                ir2  = 'ENGEL' if parts[1] == '0' else 'temiz'
                dist = parts[2]
                print(f'IR1: {ir1}  |  IR2: {ir2}  |  Mesafe: {dist} cm')

except serial.SerialException as e:
    print(f"Seri port hatasi: {e}")
except KeyboardInterrupt:
    print("\nDurduruldu.")
finally:
    try:
        ser.close()
    except:
        pass
