"""L298N + RPi GPIO test scripti - GÜVENLİ İLK TEST
Sadece kısa süreli, düşük hızda ileri hareket ile bağlantıları doğrular.
"""
import RPi.GPIO as GPIO
import time

# --- Pin tanımları (BCM) ---
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 18
ENB = 19

# --- Test parametreleri ---
TEST_HIZ = 30        # 0-100 arası PWM duty cycle, ÇOK DÜŞÜK tuttum (ilk test için)
TEST_SURE = 0.6       # saniye - çok kısa, sadece hareketi görmek için

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)  # 1kHz
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def dur():
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def ileri(hiz, sure):
    print(f"İleri hareket: hız={hiz}, süre={sure}s")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(hiz)
