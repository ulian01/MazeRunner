void setup() {
  Serial.begin(9600);     // USB to laptop
  Serial1.begin(9600);    // HC-12
  Serial.println("HUB READY");
}

void loop() {
  // Laptop -> Robot
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // Robot -> Laptop
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
