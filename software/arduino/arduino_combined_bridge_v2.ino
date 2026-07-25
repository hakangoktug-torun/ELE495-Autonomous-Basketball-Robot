/*
  ELE495 - Arduino Birlesik Sensor Bridge (v2 - BNO055 dostu surum)
  BNO055 (heading) + TCS34725 (renk) + 2x IR + HC-SR04 (ultrasonik) + Vcc
  RPi'ye 115200 baud'da CSV formatinda gonderir (FORMAT DEGISMEDI):
    IR1,IR2,Distance,Heading,R,G,B,C,VccMv

  v2 DEGISIKLIKLERI (heading tazeligi/dogrulugu icin):

  1) ULTRASONIK ARTIK HER DONGUDE OLCULMUYOR:
     pulseIn() bloklayan bir fonksiyon - echo gelmezse timeout kadar
     (eskiden 15ms!) tum donguyu kilitliyordu. Hizli modda 10ms hedeflenen
     dongu, fiilen ~25ms'lik heading periyoduna donusuyordu. Artik:
       - Mesafe SADECE ULTRASONIK_ARALIGI'nda (50ms) bir olculuyor,
         aradaki dongulerde son deger onbellekten gonderiliyor.
         (Navigasyon kodu mesafeyi zaten bu kadans yeterliligiyle kullaniyor;
         heading ise her dongude TAZE okunmaya devam ediyor.)
       - Timeout 15000us -> 9000us: saha 80x120cm, kosegen ~150cm.
         9ms ~ 154cm'ye denk gelir - platformda olculebilecek her mesafeyi
         kapsar, echo kaybolursa en fazla 9ms kaybedilir (eskiden 15ms,
         ustelik HER dongude).

  2) Vcc OLCUMU SEYRELTILDI: readVcc() icindeki delay(2), her dongude
     %20'lik (10ms'de 2ms) gereksiz bir bloklamaydi. Artik VCC_ARALIGI'nda
     (500ms) bir olculup onbellekleniyor - besleme gerilimi zaten bu
     hizdan daha yavas degisir.

  3) 'R' (reset) KOMUTU BUG DUZELTMESI: eskiden bno.begin() sonrasi
     EEPROM offsetleri GERI YUKLENMIYORDU - Python tarafi kilitlenme
     tespitinde reset gonderdiginde sensor SIFIR kalibrasyonla ayaga
     kalkiyor, sonraki tum donusler bozuk heading'le yapiliyordu.
     Artik reset sonrasi kalibrasyonuYukle() cagriliyor.

  4) IMUPLUS MODU SECENEGI (varsayilan: ACIK):
     NDOF modunda heading hesabina MAGNETOMETRE karisir - motor
     akimlarinin yarattigi manyetik girisim, RPi tarafinda filtrelemek
     zorunda kaldigimiz ani heading sicramalarinin ana kaynagi.
     Robotun ihtiyaci GORELI donus (mutlak kuzey degil), bu yuzden
     IMUPLUS (sadece gyro+ivme, mag YOK) modu:
       - motor EMI'sinden tamamen bagimsiz, cok daha temiz heading
       - mag kalibrasyonu derdi tamamen ortadan kalkar (8 cizme vs. yok)
       - bedeli: heading zamanla cok yavas kayar (gyro drift, ~1-2
         derece/dakika mertebesi) - 5 dakikalik demo icin ihmal edilebilir,
         cunku her donus zaten o anki heading'e GORELI olculuyor.
     Eski davranisa donmek icin asagidaki satiri false yap.

  KUTUPHANELER: Adafruit BNO055, Adafruit Unified Sensor, Adafruit TCS34725

  BAGLANTI (DEGISMEDI):
    BNO055   VCC->5V  GND->GND  SDA->A4  SCL->A5   (I2C 0x28)
    TCS34725 VIN->5V  GND->GND  SDA->A4  SCL->A5   (I2C 0x29)
    IR1 -> pin 4, IR2 -> pin 5, TRIG -> pin 9, ECHO -> pin 10
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_TCS34725.h>
#include <EEPROM.h>

// ================== AYARLAR ==================
const bool IMUPLUS_KULLAN = true;   // true: gyro+ivme (mag YOK, EMI bagisik)
                                     // false: eski NDOF davranisi

const unsigned long OKUMA_ARALIGI      = 50;   // ms - normal mod dongu araligi
const unsigned long HIZLI_ARALIK       = 10;   // ms - hizli mod ('F') dongu araligi
const unsigned long ULTRASONIK_ARALIGI = 50;   // ms - mesafe olcum kadansi
const unsigned long VCC_ARALIGI        = 500;  // ms - Vcc olcum kadansi
const unsigned long ULTRASONIK_TIMEOUT = 9000; // us - ~154cm (saha kosegeni ~150cm)

// ---------- EEPROM kalibrasyon ----------
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_OFFSETS_ADDR (EEPROM_MAGIC_ADDR + sizeof(long))
#define EEPROM_SAVEDCAL_ADDR (EEPROM_OFFSETS_ADDR + sizeof(adafruit_bno055_offsets_t))
const long BNO055_KAYIT_IMZASI = 0x424E4F31;  // "BNO1"

// ---------- BNO055 ----------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bnoHazir = false;

// ---------- TCS34725 ----------
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsHazir = false;

bool kalibrasyonEepromdanYuklendi = false;
uint8_t kayitliSys = 0, kayitliGyro = 0, kayitliAccel = 0, kayitliMag = 0;

// ---------- BNO055 mod secimi ----------
void bnoModunuAyarla() {
  if (IMUPLUS_KULLAN) {
    bno.setMode(OPERATION_MODE_IMUPLUS);  // gyro+ivme fuzyonu, mag KAPALI
  }
  // IMUPLUS_KULLAN=false ise begin() zaten NDOF'ta birakir, dokunmuyoruz.
}

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
  Serial.println("Kayitli kalibrasyon silindi.");
}

// ---------- TCS34725 - bloklamayan register okumasi (DEGISMEDI) ----------
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

// ---------- Pinler ----------
const int IR_PIN_1 = 4;
const int IR_PIN_2 = 5;
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

unsigned long sonOkuma = 0;
unsigned long sonUltrasonik = 0;
unsigned long sonVcc = 0;
long onbellekMesafe = -1;   // son gecerli ultrasonik olcumu
long onbellekVccMv = 0;     // son Vcc olcumu
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
    kalibrasyonuYukle();
    bnoModunuAyarla();   // IMUPLUS acildiysa burada devreye girer
    if (IMUPLUS_KULLAN) {
      Serial.println("BNO055 IMUPLUS modunda (mag kapali, motor EMI bagisik).");
    }
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

  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIK_TIMEOUT);
  if (duration == 0) {
    return -1;
  }
  return duration * 0.0343 / 2;
}

long readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  uint8_t low  = ADCL;
  uint8_t high = ADCH;

  long sonuc = (high << 8) | low;
  sonuc = 1125300L / sonuc;
  return sonuc;
}

void loop() {
  if (Serial.available()) {
    char komut = Serial.read();
    handleKomut(komut);
  }

  unsigned long simdi = millis();
  unsigned long gerekliAralik = hizliMod ? HIZLI_ARALIK : OKUMA_ARALIGI;
  if (simdi - sonOkuma < gerekliAralik) {
    return;
  }
  sonOkuma = simdi;

  // ---- IR (dijital okuma, ~us mertebesi, her dongude sorun degil) ----
  int ir1 = digitalRead(IR_PIN_1);
  int ir2 = digitalRead(IR_PIN_2);

  // ---- Ultrasonik: SADECE kadansinda olc, aksi halde onbellek ----
  if (simdi - sonUltrasonik >= ULTRASONIK_ARALIGI) {
    sonUltrasonik = simdi;
    onbellekMesafe = readUltrasonicCM();
  }

  // ---- Vcc: SADECE kadansinda olc ----
  if (simdi - sonVcc >= VCC_ARALIGI) {
    sonVcc = simdi;
    onbellekVccMv = readVcc();
  }

  // ---- BNO055 heading: HER dongude TAZE okunur ----
  float heading = -1.0;
  if (bnoHazir) {
    sensors_event_t event;
    bno.getEvent(&event);
    heading = event.orientation.x;
  }

  // ---- TCS34725 (bloklamiyor) ----
  static uint16_t r = 0, g = 0, b = 0, c = 0;
  if (tcsHazir) {
    renkOkuBloklamadan(r, g, b, c);
  }

  // ---- CSV: IR1,IR2,Distance,Heading,R,G,B,C,VccMv (FORMAT AYNI) ----
  Serial.print(ir1);            Serial.print(",");
  Serial.print(ir2);            Serial.print(",");
  Serial.print(onbellekMesafe); Serial.print(",");
  Serial.print(heading);        Serial.print(",");
  Serial.print(r);              Serial.print(",");
  Serial.print(g);              Serial.print(",");
  Serial.print(b);              Serial.print(",");
  Serial.print(c);              Serial.print(",");
  Serial.println(onbellekVccMv);
}

// 'R' -> BNO055 reset (ARTIK kalibrasyon + mod geri yukleniyor - bug fix)
// 'C' -> kalibrasyon durumu, 'S' -> EEPROM'a kaydet, 'D' -> sil
// 'F' -> hizli mod, 'N' -> normal mod, 'G' -> kayitli kalibrasyonu goster
void handleKomut(char komut) {
  if (komut == 'R' && bnoHazir) {
    bno.begin();
    delay(500);
    bno.setExtCrystalUse(true);
    kalibrasyonuYukle();   // BUG FIX: reset sonrasi offsetler geri yukleniyor
    bnoModunuAyarla();     // IMUPLUS moduna da geri don
    Serial.println("BNO055 resetlendi, kalibrasyon ve mod geri yuklendi.");
  } else if (komut == 'F') {
    hizliMod = true;
    Serial.println("Hizli mod acildi.");
  } else if (komut == 'N') {
    hizliMod = false;
    Serial.println("Normal mod.");
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
