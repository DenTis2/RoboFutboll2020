int inByte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    Serial1.write(inByte);
  }

  if (Serial1.available() > 0) {
    // get incoming byte:
    inByte = Serial1.read();
    Serial.write(inByte);
  }
  
}
