const int IR_PIN = 2;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  Serial.println("E18-D80NK IR Sensor Test Started");
}

void loop() {
  int sensorValue = digitalRead(IR_PIN);

  if (sensorValue == LOW) {
    Serial.println("OBSTACLE DETECTED");
  } else {
    Serial.println("Clear - no obstacle");
  }

  delay(200);
}