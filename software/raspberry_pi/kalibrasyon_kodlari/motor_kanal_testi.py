import RPi.GPIO as GPIO
import time

IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

try:
    print("Sadece MOTOR A (IN1/IN2/ENA) calisiyor - hangi tekerlekler donuyor bakin...")
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.LOW)  # Motor B kapali
    pwm_a.ChangeDutyCycle(40)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2)
    pwm_a.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]: GPIO.output(p, GPIO.LOW)
    time.sleep(1)

    print("Sadece MOTOR B (IN3/IN4/ENB) calisiyor - hangi tekerlekler donuyor bakin...")
    GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.LOW)  # Motor A kapali
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(40)
    time.sleep(2)
    pwm_b.ChangeDutyCycle(0)
    for p in [IN1, IN2, IN3, IN4]: GPIO.output(p, GPIO.LOW)

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("Test bitti. Hangi tekerleklerin MOTOR A'da, hangilerinin MOTOR B'de dondugunu not edin.")
