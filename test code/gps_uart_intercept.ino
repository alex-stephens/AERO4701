void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0) {
          // read the incoming byte:
          Serial.println(Serial2.readString());
  }
}
