"""
Duz gitme kalibrasyonu - sol/sag motor trim ayari
Kullanim: SOL_HIZ ve SAG_HIZ degerlerini deneye deneye ayarlayin,
araba tam duz gidene kadar. Sonra ayni HIZ ile mesafe olcumu yapin.
"""
import RPi.GPIO as GPIO
import time

IN1, IN2, IN3, IN4 = 5, 6, 13, 26
ENA, ENB = 12, 16

# ============================================================
# BURAYI AYARLAYIN
# ============================================================
HIZ = 25          # <-- rota kodunuzdaki ILERI_HIZ ile AYNI olmali
SURE = 5.0        # test suresi (saniye) - olcumu kolaylastirmak icin 1sn onerilir

# Trim: sagAAA kayiyorsa SAG_HIZ'i artirin veya SOL_HIZ'i azaltin (2-3 birimlik adimlarla)
# ENA hangi tarafi kontrol ediyorsa (sol/sag) onu SOL_HIZ, digerini SAG_HIZ yapin.
SOL_HIZ = HIZ         # ENA / pwm_a tarafi - varsayim: sol
SAG_HIZ = HIZ + 3       # ENB / pwm_b tarafi - varsayim: sag
# ============================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)

pwm_a.ChangeDutyCycle(SOL_HIZ)
pwm_b.ChangeDutyCycle(SAG_HIZ)
time.sleep(SURE)

pwm_a.ChangeDutyCycle(0)
pwm_b.ChangeDutyCycle(0)
for p in [IN1, IN2, IN3, IN4]:
    GPIO.output(p, GPIO.LOW)
GPIO.cleanup()

print(f"SOL_HIZ={SOL_HIZ}, SAG_HIZ={SAG_HIZ}, SURE={SURE}s")
print("Duz gitti mi? Hayirsa yukaridaki degerleri degistirip tekrar deneyin.")
print("Duz gittiyse: katedilen mesafeyi cetvelle olcup CM_PER_SANIYE = mesafe / SURE yapin.")
