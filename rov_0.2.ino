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
int manipulator = 80;
void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
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
  if (stringComplete) {
    char string = inputString.charAt(0);
    Serial.println(string);
    inputString.setCharAt(0,'0');
    Serial.println(inputString);
    int speed = inputString.toInt();
    Serial.println(speed);
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
     digitalWrite(6, manipulator);
     delay(200);
     digitalWrite(7, LOW);
     digitalWrite(8, LOW);
     }
     else{
     digitalWrite(7, LOW);
     digitalWrite(8, HIGH);
     digitalWrite(6, manipulator);
     delay(200);
     digitalWrite(7, LOW);
     digitalWrite(8, LOW);
     }
    }
    speed = inputString.toInt();
    //Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    digitalWrite(13, 1);
    delay(100);
    digitalWrite(13, 0);     
  }
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
