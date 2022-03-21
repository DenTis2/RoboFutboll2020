#include <Servo.h>

Servo myservo1;

void setup() {
  // put your setup code here, to run once:
  myservo1.attach(6);
}

void loop() {
  // put your main code here, to run repeatedly:
  myservo1.write(168);
  delay(2000);
  myservo1.write(1);
  delay(2000);
  
  myservo1.write(15);
  delay(200);
  myservo1.write(1);
  delay(200);
  myservo1.write(15);
  delay(200);
  myservo1.write(1);
  delay(200);
  myservo1.write(15);
  delay(200);
  myservo1.write(1);
  delay(200);
}
