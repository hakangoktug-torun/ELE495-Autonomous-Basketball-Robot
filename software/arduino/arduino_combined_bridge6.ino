/*
  ELE495 - Arduino Birlesik Sensor Bridge
  BNO055 (heading) + TCS34725 (renk) + 2x IR + HC-SR04 (ultrasonik) + Vcc
  RPi'ye 115200 baud'da CSV formatinda gonderir:
    IR1,IR2,Distance,Heading,R,G,B,C,VccMv

  NOT: TCS34725 (renk sensoru) BLOKLAMAYAN bir yontemle okunuyor
  (renkOkuBloklamadan fonksiyonu, dogrudan I2C register okumasi) - kutuphanenin
  standart getRawData() fonksiyonu her cagrildiginda ~50ms bekliyordu, bu da
  BNO055 heading guncellemesini yavaslatiyordu. Artik renk sensoru heading
  hizini HIC etkilemiyor, guvenle her dongude okunabiliyor.

  KUTUPHANELER (Arduino IDE Library Manager'dan kur):
    - Adafruit BNO055
    - Adafruit Unified Sensor
    - Adafruit TCS34725

  BAGLANTI:
    BNO055   VCC->5V  GND->GND  SDA->A4  SCL->A5   (I2C adres 0x28)
    TCS34725 VIN->5V  GND->GND  SDA->A4  SCL->A5   (I2C adres 0x29, ayni hat, cakisma yok)
    IR1 -> pin 4
    IR2 -> pin 5
    TRIG -> pin 9
    ECHO -> pin 10
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_TCS34725.h>
#include <EEPROM.h>

// ---------- EEPROM kalibrasyon kalici depolama ----------
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_OFFSETS_ADDR (EEPROM_MAGIC_ADDR + sizeof(long))
#define EEPROM_SAVEDCAL_ADDR (EEPROM_OFFSETS_ADDR + sizeof(adafruit_bno055_offsets_t))
const long BNO055_KAYIT_IMZASI = 0x424E4F31;  // "BNO1" - kayitli veri gecerli mi kontrolu icin

// ---------- BNO055 ----------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bnoHazir = false;

// ---------- TCS34725 ----------
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsHazir = false;

// ---------- EEPROM kalibrasyon fonksiyonlari ----------
// NOT: Bu fonksiyonlar 'bno' nesnesi tanimlandiktan SONRA gelmeli, yoksa
// derleyici 'bno' bulunamadi hatasi verir.

bool kalibrasyonEepromdanYuklendi = false;
uint8_t kayitliSys = 0, kayitliGyro = 0, kayitliAccel = 0, kayitliMag = 0;

void kalibrasyonuYukle() {
  long imza;
  EEPROM.get(EEPROM_MAGIC_ADDR, imza);

  if (imza == BNO055_KAYIT_IMZASI) {
    adafruit_bno055_offsets_t offsets;
    EEPROM.get(EEPROM_OFFSETS_ADDR, offsets);
    bno.setSensorOffsets(offsets);
    kalibrasyonEepromdanYuklendi = true;

    EEPROM.get(EEPROM_SAVEDCAL_ADDR, kayitliSys);
    EEPROM.get(EEPROM_SAVEDCAL_ADDR + 1, kayitliGyro);
    EEPROM.get(EEPROM_SAVEDCAL_ADDR + 2, kayitliAccel);
    EEPROM.get(EEPROM_SAVEDCAL_ADDR + 3, kayitliMag);

    Serial.println("Kayitli BNO055 kalibrasyonu EEPROM'dan yuklendi.");
  } else {
    Serial.println("Kayitli kalibrasyon bulunamadi, sifirdan kalibre edilmeli.");
  }
}

void kalibrasyonuKaydet() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);
  EEPROM.put(EEPROM_OFFSETS_ADDR, offsets);
  EEPROM.put(EEPROM_MAGIC_ADDR, BNO055_KAYIT_IMZASI);

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  EEPROM.put(EEPROM_SAVEDCAL_ADDR, sys);
  EEPROM.put(EEPROM_SAVEDCAL_ADDR + 1, gyro);
  EEPROM.put(EEPROM_SAVEDCAL_ADDR + 2, accel);
  EEPROM.put(EEPROM_SAVEDCAL_ADDR + 3, mag);
  kayitliSys = sys; kayitliGyro = gyro; kayitliAccel = accel; kayitliMag = mag;

  Serial.println("BNO055 kalibrasyonu EEPROM'a kaydedildi.");
}

void kalibrasyonuSil() {
  long gecersiz_imza = 0;
  EEPROM.put(EEPROM_MAGIC_ADDR, gecersiz_imza);
  kalibrasyonEepromdanYuklendi = false;
  kayitliSys = kayitliGyro = kayitliAccel = kayitliMag = 0;
  Serial.println("Kayitli kalibrasyon silindi. Bir sonraki acilista sifirdan kalibre edilecek.");
}

// ---------- TCS34725 (renk) - BLOKLAMAYAN dogrudan register okumasi ----------
// NOT: Adafruit kutuphanesinin getRawData() fonksiyonu her cagrildiginda
// entegrasyon suresi kadar (50ms) bekler - bu da BNO055 heading guncellemesini
// yavaslatiyordu (daha once bu yuzden 'hizli mod'da renk sensorunu TAMAMEN
// kapatmistik). Ama TCS34725 cipi, bir kere etkinlestirildikten (PON+AEN)
// SONRA arka planda SUREKLI entegrasyon yapip veri kayitlarini kendiliginden
// gunceller - yani her okuma icin yeniden beklememize GEREK YOK. Kayitlari
// dogrudan I2C uzerinden okuyoruz, bu islem ~1ms'den kisa surer (BNO055
// okumasi kadar hizli), 50ms'lik bekleme tamamen ortadan kalkar.
#define TCS34725_I2C_ADDR    0x29
#define TCS34725_CMD_BIT     0x80
#define TCS34725_REG_CDATAL  0x14
#define TCS34725_REG_RDATAL  0x16
#define TCS34725_REG_GDATAL  0x18
#define TCS34725_REG_BDATAL  0x1A

uint16_t tcsRegisterOku16(uint8_t reg) {
  Wire.beginTransmission(TCS34725_I2C_ADDR);
  Wire.write(TCS34725_CMD_BIT | reg);
  Wire.endTransmission();
  Wire.requestFrom(TCS34725_I2C_ADDR, 2);
  uint16_t dusuk = Wire.read();
  uint16_t yuksek = Wire.read();
  return (yuksek << 8) | dusuk;
}

void renkOkuBloklamadan(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  c = tcsRegisterOku16(TCS34725_REG_CDATAL);
  r = tcsRegisterOku16(TCS34725_REG_RDATAL);
  g = tcsRegisterOku16(TCS34725_REG_GDATAL);
  b = tcsRegisterOku16(TCS34725_REG_BDATAL);
}

// ---------- IR ve Ultrasonik pinleri ----------
const int IR_PIN_1 = 4;
const int IR_PIN_2 = 5;
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

const unsigned long OKUMA_ARALIGI = 50; // ms
unsigned long sonOkuma = 0;

// HIZLI MOD: SADECE genel dongu bekleme araligini (OKUMA_ARALIGI) etkiler,
// renk sensoruyle artik ILGISI YOK (renk okumasi artik bloklamiyor, bkz.
// renkOkuBloklamadan). RPi 'F' komutuyla dongu araligini 50ms'den 10ms'ye
// dusurup heading guncelleme hizini biraz daha artirabilir, ama artik renk
// sensorunu kapatmaya gerek olmadigi icin bu opsiyonel bir ince ayar.
bool hizliMod = false;

void setup() {
  Serial.begin(115200);

  pinMode(IR_PIN_1, INPUT);
  pinMode(IR_PIN_2, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();

  bnoHazir = bno.begin();
  if (!bnoHazir) {
    Serial.println("HATA: BNO055 bulunamadi, baglantilari kontrol et");
  } else {
    delay(1000);
    bno.setExtCrystalUse(true);
    kalibrasyonuYukle();  // daha once kaydedilmis kalibrasyon varsa yukle
  }

  tcsHazir = tcs.begin();
  if (!tcsHazir) {
    Serial.println("HATA: TCS34725 bulunamadi, baglantilari kontrol et");
  }

  delay(200);
  while (Serial.available()) Serial.read();
}

long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 15000); // ~250cm yeterli
  if (duration == 0) {
    return -1;
  }
  return duration * 0.0343 / 2;
}

long readVcc() {
  // Arduino Uno (ATmega328P) icin: dahili 1.1V referansini, Vcc'yi (5V hatti)
  // ADC referansi olarak kullanarak olcer - yani "5V'a gore 1.1V ne kadar"
  // yerine, "Vcc bilinmezken, bilinen 1.1V referansi ADC'de kac sayiya denk
  // geliyor" mantigiyla TERSINE cevirip gercek Vcc'yi hesapliyoruz.
  // Hicbir harici bilesen/pin gerekmez.
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // referansin oturmasi icin kisa bekleme
  ADCSRA |= _BV(ADSC); // donusumu baslat
  while (bit_is_set(ADCSRA, ADSC)); // olcum bitene kadar bekle

  uint8_t low  = ADCL; // once ADCL okunmali - ADCH'i kilitler
  uint8_t high = ADCH; // ikisini de acar

  long sonuc = (high << 8) | low;
  sonuc = 1125300L / sonuc; // Vcc (mV) = 1.1 * 1023 * 1000 / ADC_degeri
  return sonuc; // mV cinsinden Vcc
}

void loop() {
  // ---- RPi'den gelen komutlari kontrol et ----
  if (Serial.available()) {
    char komut = Serial.read();
    handleKomut(komut);
  }

  unsigned long simdi = millis();
  unsigned long gerekliAralik = hizliMod ? 10 : OKUMA_ARALIGI;
  if (simdi - sonOkuma < gerekliAralik) {
    return;
  }
  sonOkuma = simdi;

  // ---- IR ve ultrasonik ----
  int ir1 = digitalRead(IR_PIN_1);
  int ir2 = digitalRead(IR_PIN_2);
  long distanceCM = readUltrasonicCM();

  // ---- BNO055 heading ----
  float heading = -1.0;
  if (bnoHazir) {
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;
  }

  // ---- TCS34725 renk (artik BLOKLAMIYOR - her dongude okunabilir) ----
  static uint16_t r = 0, g = 0, b = 0, c = 0;
  if (tcsHazir) {
    renkOkuBloklamadan(r, g, b, c);
  }

  // ---- Vcc (5V hatti) gerilimi - guc yetersizligini tespit etmek icin ----
  long vccMv = readVcc();

  // ---- CSV: IR1,IR2,Distance,Heading,R,G,B,C,VccMv ----
  Serial.print(ir1);         Serial.print(",");
  Serial.print(ir2);         Serial.print(",");
  Serial.print(distanceCM);  Serial.print(",");
  Serial.print(heading);     Serial.print(",");
  Serial.print(r);           Serial.print(",");
  Serial.print(g);           Serial.print(",");
  Serial.print(b);           Serial.print(",");
  Serial.print(c);           Serial.print(",");
  Serial.println(vccMv);
}

// 'R' -> BNO055 sistem resetini tetikle
// 'C' -> anlik kalibrasyon durumunu bildir (sys,gyro,accel,mag)
// 'S' -> mevcut kalibrasyonu EEPROM'a kalici olarak kaydet
// 'D' -> kayitli kalibrasyonu sil (bir sonraki acilista sifirdan kalibre edilir)
// 'F' -> HIZLI MOD: dongu bekleme araligini kisaltir (renk sensoru artik bloklamiyor, ondan bagimsiz)
// 'N' -> NORMAL MOD: dongu bekleme araligini normale dondurur
void handleKomut(char komut) {
  if (komut == 'R' && bnoHazir) {
    bno.begin();
    delay(500);
    bno.setExtCrystalUse(true);
  } else if (komut == 'F') {
    hizliMod = true;
    Serial.println("Hizli mod acildi - dongu araligi kisaltildi (renk sensoru zaten etkilenmiyor).");
  } else if (komut == 'N') {
    hizliMod = false;
    Serial.println("Normal mod - dongu araligi normale dondu.");
  } else if (komut == 'C' && bnoHazir) {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("CAL,");
    Serial.print(sys);   Serial.print(",");
    Serial.print(gyro);  Serial.print(",");
    Serial.print(accel); Serial.print(",");
    Serial.print(mag);   Serial.print(",");
    Serial.println(kalibrasyonEepromdanYuklendi ? 1 : 0);
  } else if (komut == 'S' && bnoHazir) {
    kalibrasyonuKaydet();
  } else if (komut == 'D') {
    kalibrasyonuSil();
  } else if (komut == 'G') {
    Serial.print("SAVEDCAL,");
    Serial.print(kayitliSys);   Serial.print(",");
    Serial.print(kayitliGyro);  Serial.print(",");
    Serial.print(kayitliAccel); Serial.print(",");
    Serial.print(kayitliMag);   Serial.print(",");
    Serial.println(kalibrasyonEepromdanYuklendi ? 1 : 0);
  }
}
