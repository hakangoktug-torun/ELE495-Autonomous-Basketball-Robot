import RPi.GPIO as GPIO
import serial
import time
import threading

# --- MOTOR PİNLERİ ---
IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16

# --- KALİBRASYON DEĞERLERİ ---
ILERI_HIZ = 25
DONUS_HIZ = 30
SURE_TAM_TUR = 3.15

# --- SENSÖR AYARLARI ---
PORT = '/dev/ttyUSB0'
BAUD = 115200
DURMA_MESAFESI = 15  # cm

# --- GLOBAL MESAFE DEĞİŞKENİ ---
current_distance = 999
distance_lock = threading.Lock()

# --- SERİ OKUMA THREAD'İ ---
def serial_reader():
    global current_distance
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        dist = int(parts[2])
                        if dist > 0:
                            with distance_lock:
                                current_distance = dist
                    except ValueError:
                        pass
    except Exception as e:
        print(f"Seri port hatasi: {e}")

def get_distance():
    with distance_lock:
        return current_distance

# --- GPIO KURULUM ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

# --- MOTOR FONKSİYONLARI ---
def dur(bekle=0.3):
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.output(p, GPIO.LOW)
    time.sleep(bekle)

def ileri_adim(hiz):
    """Süre vermeden sadece ileri hareket başlatır, dur çağrılana kadar gider."""
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)

def kendi_ekseninde_don(sure, hiz):
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)
    time.sleep(sure)
    dur()

def geri(sure, hiz):
    GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)
    time.sleep(sure)
    dur()

# --- ANA PROGRAM ---
try:
    # Sensör thread'ini başlat
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    print("Sensör bekleniyor... 3 saniye")
    time.sleep(3)

    # İlk mesafeyi kontrol et
    dist = get_distance()
    print(f"Baslangic mesafesi: {dist} cm")

    if dist <= DURMA_MESAFESI:
        print("Engel zaten cok yakin! Hareket iptal.")
    else:
        # İleri git, sürekli mesafe kontrol et
        print(f"1) Ileri gidiliyor... (Durma mesafesi: {DURMA_MESAFESI} cm)")
        ileri_adim(ILERI_HIZ)

        while True:
            dist = get_distance()
            print(f"   Mesafe: {dist} cm")

            if dist <= DURMA_MESAFESI:
                dur()
                print(f"Engel algilandi! {dist} cm'de duruldu.")
                break

            time.sleep(0.02)  # 20ms'de bir kontrol et

        print(f"2) Kendi ekseninde 360 derece donus (HIZ={DONUS_HIZ})")
        kendi_ekseninde_don(SURE_TAM_TUR, DONUS_HIZ)

        print("Test tamamlandi.")

except KeyboardInterrupt:
    print("Kullanici tarafindan durduruldu.")
finally:
    dur(0)
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("GPIO temizlendi.")
