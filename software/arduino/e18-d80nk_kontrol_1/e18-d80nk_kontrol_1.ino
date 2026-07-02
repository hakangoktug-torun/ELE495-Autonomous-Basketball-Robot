const int IR_PIN_1 = 4;
const int IR_PIN_2 = 5;

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

void setup() {
  Serial.begin(115200); // RPi ile hızlı haberleşme için 115200 baudrate idealdir

  pinMode(IR_PIN_1, INPUT);
  pinMode(IR_PIN_2, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 15000 us (~250 cm) yeterlidir, 10cm'i yakalayacağımız için uzun beklemeye gerek yok
  long duration = pulseIn(ECHO_PIN, HIGH, 15000); 

  if (duration == 0) {
    return -1; 
  }

  long distanceCM = duration * 0.0343 / 2;
  return distanceCM;
}

void loop() {
  int ir1 = digitalRead(IR_PIN_1); // LOW = Engel Var (0), HIGH = Temiz (1)
  int ir2 = digitalRead(IR_PIN_2);
  long distanceCM = readUltrasonicCM();

  // RPi için veri formatı: "IR1,IR2,Mesafe" -> Örn: "1,1,45" veya "0,1,8"
  Serial.print(ir1);
  Serial.print(",");
  Serial.print(ir2);
  Serial.print(",");
  Serial.println(distanceCM);

  delay(50); // Saniyede 20 defa veri göndermek RPi ve motor tepkisi için fazlasıyla yeterli
}