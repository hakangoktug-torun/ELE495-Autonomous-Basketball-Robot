#include <WiFiS3.h>

// ---- WiFi ayarlari ----
const char* WIFI_SSID = "TORCUK'S Galaxy Note10 Lite";     // kendi hotspot adinizi yazin
const char* WIFI_PASS = "sdkk8211";          // kendi sifrenizi yazin
const char* RPI_IP     = "10.144.149.91";           // RPi'nin guncel IP'si (statik IP atanana kadar her hotspot baglantisinda kontrol edin)
const int   RPI_PORT   = 5000;                       // Flask varsayilan portu

WiFiClient client;

// ---- Sensor pinleri ----
const int BEAM_PIN_1 = 2; // interrupt destekli
const int BEAM_PIN_2 = 3; // interrupt destekli
const int BEAM_PIN_3 = 4; // polling
const int BEAM_PIN_4 = 5; // polling

volatile bool engelVarMi1 = false;
volatile bool engelVarMi2 = false;

bool sensor3OncekiDurum = true;
bool sensor4OncekiDurum = true;

unsigned long sonDurumYazdirma = 0;
const unsigned long DURUM_ARALIGI = 200;

// ---- Cooldown (GLOBAL - top gecis suresine gore ayarlanabilir) ----
// GUNCELLEME: 400ms -> 1500ms. sonBildirimZamani TEK bir degisken oldugu
// icin (sensore ozel DEGIL), herhangi bir sensor (1, 2, 3 ya da 4 fark
// etmeksizin) topu algilayip bildirim gonderdiginde, bu zaman damgasi
// GUNCELLENIR ve ardindan gelen COOLDOWN_MS suresi boyunca DIGER TUM
// sensorlerin tetiklemeleri de (asagidaki tek "if (simdi - sonBildirimZamani
// >= COOLDOWN_MS)" kontrolu sayesinde) YOK SAYILIR. Yani ayni topun
// cembere girerken 1 sensoru, cikarken/sekerek baska bir sensoru
// tetiklemesi durumunda, ikisi AYRI ayri sayilmaz - tek bir gecis olarak
// islenir.
unsigned long sonBildirimZamani = 0;
const unsigned long COOLDOWN_MS = 1500; // 1.5 saniye - TUM sensorler icin GECERLI, tek sensore ozel degil

void isr_engel1() { engelVarMi1 = true; }
void isr_engel2() { engelVarMi2 = true; }

void baglanWiFi() {
  Serial.print("WiFi'ye baglaniliyor: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int deneme = 0;
  while (WiFi.status() != WL_CONNECTED && deneme < 30) { // max ~15 saniye
    delay(500);
    Serial.print(".");
    deneme++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Baglandi! IP adresim: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("BAGLANAMADI! Durum kodu: ");
    Serial.println(WiFi.status());
    // 1 = WL_IDLE_STATUS, 4 = WL_CONNECT_FAILED, 6 = WL_DISCONNECTED vs.
  }
}

void topGectiBildir(int sensorNo) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi bagli degil, bildirim gonderilemedi!");
    return;
  }

  Serial.print("RPi'ye bildiriliyor... Sensor: ");
  Serial.println(sensorNo);

  if (client.connect(RPI_IP, RPI_PORT)) {
    String istek = "GET /skor?sensor=" + String(sensorNo) + " HTTP/1.1\r\n";
    istek += "Host: " + String(RPI_IP) + "\r\n";
    istek += "Connection: close\r\n\r\n";
    client.print(istek);

    unsigned long baslangic = millis();
    while (client.connected() && millis() - baslangic < 300) {
      if (client.available()) {
        client.read();
      }
    }
    client.stop();
    Serial.println("Bildirim gonderildi.");
  } else {
    Serial.println("RPi'ye baglanilamadi! IP/port kontrol edin.");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BEAM_PIN_1, INPUT);
  pinMode(BEAM_PIN_2, INPUT);
  pinMode(BEAM_PIN_3, INPUT);
  pinMode(BEAM_PIN_4, INPUT);

  attachInterrupt(digitalPinToInterrupt(BEAM_PIN_1), isr_engel1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BEAM_PIN_2), isr_engel2, FALLING);

  baglanWiFi();

  Serial.println("=== Sistem hazir, top gecisi bekleniyor ===");
}

void loop() {
  bool topAlgilandiBuTur = false;
  int tetiklenenSensor = -1;

  // ---- 4 sensorun HEPSI kontrol edilir - hangisi tetiklenirse tetiklensin
  // ayni "topAlgilandiBuTur" bayragini set eder, asagidaki TEK global
  // cooldown kontrolune tabi olur. ----
  if (engelVarMi1) {
    topAlgilandiBuTur = true;
    tetiklenenSensor = 1;
    engelVarMi1 = false;
  }
  if (engelVarMi2) {
    topAlgilandiBuTur = true;
    tetiklenenSensor = 2;
    engelVarMi2 = false;
  }

  bool sensor3Durum = digitalRead(BEAM_PIN_3) == HIGH;
  if (sensor3Durum != sensor3OncekiDurum) {
    if (!sensor3Durum) {
      topAlgilandiBuTur = true;
      tetiklenenSensor = 3;
    }
    sensor3OncekiDurum = sensor3Durum;
  }

  bool sensor4Durum = digitalRead(BEAM_PIN_4) == HIGH;
  if (sensor4Durum != sensor4OncekiDurum) {
    if (!sensor4Durum) {
      topAlgilandiBuTur = true;
      tetiklenenSensor = 4;
    }
    sensor4OncekiDurum = sensor4Durum;
  }

  // ---- GLOBAL COOLDOWN KONTROLU ----
  // sonBildirimZamani TUM sensorler icin ORTAK - bu yuzden 1. sensor
  // tetiklenip bildirim gonderdikten sonra, 2/3/4. sensorlerden HERHANGI
  // BIRI COOLDOWN_MS (1.5s) icinde tetiklense bile bu blok calismaz,
  // bildirim gonderilmez - ayni topun sekerek/gecerek baska bir sensoru
  // tetiklemesi ayri bir gecis olarak SAYILMAZ.
  if (topAlgilandiBuTur) {
    unsigned long simdi = millis();
    if (simdi - sonBildirimZamani >= COOLDOWN_MS) {
      Serial.print(">>> TOP ALGILANDI - Sensor: ");
      Serial.println(tetiklenenSensor);
      topGectiBildir(tetiklenenSensor);
      sonBildirimZamani = simdi;
    } else {
      unsigned long kalanMs = COOLDOWN_MS - (simdi - sonBildirimZamani);
      Serial.print("(Cooldown aktif, ");
      Serial.print(kalanMs);
      Serial.print("ms kaldi - Sensor ");
      Serial.print(tetiklenenSensor);
      Serial.println(" tetiklemesi sayilmadi, muhtemelen ayni topun sekmesi/baska sensorden gecisi)");
    }
  }

  unsigned long simdi = millis();
  if (simdi - sonDurumYazdirma >= DURUM_ARALIGI) {
    sonDurumYazdirma = simdi;
    Serial.print("S1:"); Serial.print(digitalRead(BEAM_PIN_1) ? "OK " : "YOK ");
    Serial.print("S2:"); Serial.print(digitalRead(BEAM_PIN_2) ? "OK " : "YOK ");
    Serial.print("S3:"); Serial.print(sensor3Durum ? "OK " : "YOK ");
    Serial.print("S4:"); Serial.println(sensor4Durum ? "OK" : "YOK");
  }
}
