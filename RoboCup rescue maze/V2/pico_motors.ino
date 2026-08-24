#include <Wire.h>
MbedI2C I2C(20,21);
int t=1;
void setup() {
  I2C.begin();
  delay(3000);
}

void loop() {
  /*if(t){
  I2C.beginTransmission(8);
  I2C.write("r");        // sends five bytes
  I2C.write(234);              // sends one byte
  I2C.write("br");        // sends five bytes
  I2C.write(234);              // sends one byte
  I2C.write("l");        // sends five bytes
  I2C.write(234);              // sends one byte
  I2C.write("ql");        // sends five bytes
  I2C.endTransmission();    // stop transmitting
  t=0;
  }*/
  // put your main code here, to run repeatedly:
  I2C.beginTransmission(8);
  I2C.write("r");        // sends five bytes
  I2C.write(234);              // sends one byte
  I2C.write("qr");        // sends five bytes
  I2C.endTransmission();    // stop transmitting
  delay(500);
}
