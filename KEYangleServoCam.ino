#include <Servo.h>
Servo servo1;
int angleServoCam = 90;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
servo1.attach(6);
servo1.write(angleServoCam);
}

void loop() {
  // put your main code here, to run repeatedly:
 int keyUp = digitalRead(10);
 int keyDn = digitalRead(11);
 int keyCenter = digitalRead(9);
 Serial.println(angleServoCam);
 delay(50);
 if(keyUp == 1){
  if(angleServoCam < 175) angleServoCam = angleServoCam + 2;
  servo1.write(angleServoCam);
  delay(30);
 }
 else if(keyDn == 1){
  if(angleServoCam > 4) angleServoCam = angleServoCam - 2;
  servo1.write(angleServoCam);  
  delay(30); 
 }
 if(keyCenter == 1){
  angleServoCam = 90;
  servo1.write(angleServoCam);  
  delay(50);  
 }
}
