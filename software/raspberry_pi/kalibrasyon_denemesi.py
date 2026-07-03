"""Sadece ileri hareketi kalibre et"""
import RPi.GPIO as GPIO
import time

IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16
HIZ = 50
SURE = 0.23   # ← bunu deneye deneye ayarlayacaksınız

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000); pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0); pwm_b.start(0)

GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
pwm_a.ChangeDutyCycle(HIZ); pwm_b.ChangeDutyCycle(HIZ)
time.sleep(SURE)
pwm_a.ChangeDutyCycle(0); pwm_b.ChangeDutyCycle(0)
for p in [IN1, IN2, IN3, IN4]: GPIO.output(p, GPIO.LOW)

GPIO.cleanup()
print(f"{SURE} saniyede kac cm gitti, cetvelle olcun.")
