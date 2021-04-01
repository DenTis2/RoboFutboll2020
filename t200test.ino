#include <Servo.h>
int joystiky = 0;
int signal;
byte servoPin = 9;
Servo servo;

void setup() {
  servo.attach(servoPin);
  Serial.begin(9600);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  joystiky = analogRead(A0);
  signal =   joystiky * 0.78 + 1100;
  Serial.println (signal);   
  servo.writeMicroseconds(signal); // Send signal to ESC.
  delay(50);
}

