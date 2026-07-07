"""Duz ilerle -> kendi etrafinda 360 don -> ayni yoldan geri don"""
import RPi.GPIO as GPIO
import time

IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16

# --- KALİBRASYON DEĞERLERİ (ölçülmüş) ---
ILERI_HIZ = 50
SURE_10CM = 0.23        # HIZ=50'de 10cm icin olculdu

DONUS_HIZ = 30
SURE_TAM_TUR = 3.0      # HIZ=30'da 360 derece icin olculdu

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def dur(bekle=0.5):
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]:
        GPIO.output(p, GPIO.LOW)
    time.sleep(bekle)  # motorlar tam dursun, savrulma olmasin

def ileri(sure, hiz):
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)   # sol ileri
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)  # sag ileri
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)
    time.sleep(sure)
    dur()

def geri(sure, hiz):
    GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)  # sol geri
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)   # sag geri
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)
    time.sleep(sure)
    dur()

def kendi_ekseninde_don(sure, hiz):
    # sol geri, sag ileri -> saat yonunde kendi ekseninde donus
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(hiz)
    pwm_b.ChangeDutyCycle(hiz)
    time.sleep(sure)
    dur()

try:
    print("Test basliyor... 3 saniye sonra hareket baslayacak.")
    time.sleep(3)

    print(f"1) Duz ileri ~10cm (HIZ={ILERI_HIZ}, {SURE_10CM}s)")
    ileri(SURE_10CM, ILERI_HIZ)

    print(f"2) Kendi ekseninde 360 derece donus (HIZ={DONUS_HIZ}, {SURE_TAM_TUR}s)")
    kendi_ekseninde_don(SURE_TAM_TUR, DONUS_HIZ)

    print(f"3) Geldigi yoldan geri ~10cm (HIZ={ILERI_HIZ}, {SURE_10CM}s)")
    geri(SURE_10CM, ILERI_HIZ)

    print("Test tamamlandi.")

except KeyboardInterrupt:
    print("Kullanici tarafindan durduruldu.")

finally:
    dur(0)
    pwm_a.stop(); pwm_b.stop()
    GPIO.cleanup()
    print("GPIO temizlendi.")
