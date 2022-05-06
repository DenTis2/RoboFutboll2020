#include <Servo.h>

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
Servo servoA;
Servo servoB;
Servo servoC;
Servo servoD;
int x=90;
int C = 11;
int A = 9;
int B = 10;
int D = 5;
int datchik_protechki = A0;
int laser = 12;
int manipulator = 85;
void setup() {
  // initialize serial:
  pinMode(laser,OUTPUT);
  Serial.begin(9600);
  // reserve 400 bytes for the inputString:
  inputString.reserve(400);
  servoA.attach(A);
  servoB.attach(B);
  servoC.attach(C);
  servoD.attach(D);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  servoA.writeMicroseconds(1500);
  servoB.writeMicroseconds(1500);
  servoC.writeMicroseconds(1500);
  pinMode(13, OUTPUT);
  delay(500);
}

void loop() {
  // print the string when a newline arrives:
   int s = analogRead(datchik_protechki);
   for(int i = 0; i <= 5; i++){
    s = s + analogRead(A0);
    delay(10);
   }
   s = s/5;
   if (s > 500){
    digitalWrite(laser,1);
    delay(1000);
   }
   if (stringComplete) {
    char string = inputString.charAt(0);
    //Serial.println(string);
    inputString.setCharAt(0,'0');
    //Serial.println(inputString);
    int speed = inputString.toInt();
    //Serial.println(speed);
    if(string == 'A'){
     servoA.writeMicroseconds(speed);
    }
    if(string == 'B'){
     servoB.writeMicroseconds(speed);
    }
    if(string == 'C'){
     servoC.writeMicroseconds(speed);
    }
    if(string == 'D'){
     servoD.write(speed);
     delay(50);
    }
    if(string == 'E'){
     if(speed == manipulator){
     digitalWrite(7, HIGH);
     digitalWrite(8, LOW);
     analogWrite(6, manipulator);
     delay(200);
     digitalWrite(7, LOW);
     digitalWrite(8, LOW);
     }
     else{
     digitalWrite(7, LOW); 
     digitalWrite(8, HIGH);
     analogWrite(6, manipulator);
     delay(200);
     digitalWrite(7, LOW);
     digitalWrite(8, LOW);
     }
    }
    //speed = inputString.toInt();
    //Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    digitalWrite(13, 1);
    delay(30);
    digitalWrite(13, 0);     
  }
  if(millis() / 100 % 50 == 0) digitalWrite(13, 0);
  if(millis() / 100 % 100 == 0) digitalWrite(13, 1);
}
  
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
