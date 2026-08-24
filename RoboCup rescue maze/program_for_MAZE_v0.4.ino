#include <Arduino.h>                                             
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0
#define MAXSPEED 90
//Servo servo1;
//String inputString = "";
int random_port = A7;
int variant;
int Random;
int symbol;
int Global_hand;
int t = 5;
int y;
int i;
int zalip = 0;
int w;
int storona;
int knopka = A13;
int angel;
int gyro;
int lastgyro;
int diff;
int trigPin = 60; // назначаем имя для Pin8
int echoPin = 65; // назначаем имя для Pin9
int ultra9;
float dis_front;
float dis_left;
float dis_right;
float dis_left_b;//A15
float dis_right_b;//A10
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int htColorSensorAddress = 0x01; //we got this from I2C Scanner
int TestValue;
byte ColorRed, ColorGreen, ColorBlue;
float H,S,V;
float BlackV = 7.5;
float BlueH = 180;


/**
 * This define returns the smaller of the two numbers
 */
#define min2(a, b) (a < b ? a : b)

/**
 * This define returns the smallest of the three numbers
 */
#define min3(a, b, c) (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c)

/**
 * This function returns the bigger of the two numbers
 */
#define max2(a, b) (a > b ? a : b)

/**
 * This function returns the biggest of the three numbers
 */
#define max3(a, b, c) (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c)

/**
 * Returns x if it is between min and max. If outside the range,
 * it returns min or max.
 */
#define clip(a, b, c) min2(c, max2(b, a))

MeGyro gyro_0(0, 0x69);
MeLightSensor lightsensor_12(12);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
//MeSmartServo Servo1(9);
MeColorSensor colorsensor_7(7);
MeUltrasonicSensor ultrasonic_10(10);
MeUltrasonicSensor ultrasonic_9(9);
//MeLightSensor lightsensor_12(12);

Servo servo1;
#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring( 0, 12 );
#endif
void rotate_left(){
 move(3, 70);
}
void rotate_right(){
 move(4, 100);
}
void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}
float dis_front_read(void);

/* void serialEvent() {
  int i,y;
  if(!stringComplete)
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
      led_ring.show();      
      for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 0, 255);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
    }
  }
}*/
void controlLight(MeColorSensor colorSensor, uint8_t lightState){
  if(lightState == 0){
    colorSensor.TurnOffLight();
  }else{
    colorSensor.TurnOnLight();
  }
}

uint8_t detectedColor(MeColorSensor colorSensor, uint8_t colorType){
  if(colorType == colorSensor.Returnresult()){
    return 1;
  }
  return 0;
}
void examination(){
 if(dis_front < 5 || dis_front > 50){
  dis_front = 50;
 }
 if(dis_left < 0 || dis_left > 50){
  dis_left = 50;
 }
 if(dis_right < 0 || dis_right > 50){
  dis_right = 50;
 }
 if(dis_left_b < 0 || dis_left > 50){
  dis_left_b = 50;
 }
 if(dis_right_b < 0 || dis_right > 50){
  dis_right_b = 50;
 }
}
void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }else{
    Encoder_2.pulsePosPlus();
  }
}
float IR(int p){
 float volts = analogRead(p);
 return 4200 / (volts - 57);
}
void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if(direction == 1){
    leftSpeed = -speed;
    rightSpeed = speed;
  }else if(direction == 2){
    leftSpeed = speed;
    rightSpeed = -speed;
  }else if(direction == 3){ //left
    leftSpeed = 0;
    rightSpeed = -speed;
  }else if(direction == 4){ //right
    leftSpeed = speed;
    rightSpeed = 0;
  }
  Encoder_1.setTarPWM(leftSpeed);
  Encoder_2.setTarPWM(rightSpeed);
}

void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) loop();
}

void gyro_rotate(int side){ // 0 - поворот налево, 1 - поворот направо, 2 - поврот прямо(проезд прямо)
  if(side == 0){
     led_ring.setColor( 6, 100, 0, 0);
     led_ring.show();
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.075);
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.1);
     if(dis_front <= 15){
     Encoder_1.setTarPWM(-80);
     Encoder_2.setTarPWM(80);
     _delay(0.4);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     angel = lastgyro - 45;
     if(angel < -180){
      while(gyro > -170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_left();
      _delay(0.05);
      }
      gyro_0.update();
      while(gyro > 180 + (angel + 180) || gyro > -180 && gyro < -170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_left();
      _delay(0.05);
      }
     }
     else{
      while(gyro > angel){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_left();
      _delay(0.05);
      }
     }
    move(3,0);
    delay(100);
     Encoder_1.setTarPWM(-1 * -25/100.0*255);
     Encoder_2.setTarPWM(-25/100.0*255);
     _delay(1);    
    }
  if(side == 1){
     led_ring.setColor( 9, 100, 0, 0);
     led_ring.show();
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     
     
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.1);
     if(dis_front <= 15){
     Encoder_1.setTarPWM(-80);
     Encoder_2.setTarPWM(80);
     _delay(0.4);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     angel = lastgyro + 45;
     if(angel > 180){
      while(gyro < 170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_right();
      _delay(0.05);
      gyro_0.update();
      }
      while(gyro < -180 + (angel - 180) || gyro > 170 && gyro < 180){
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
       rotate_right();
      _delay(0.05);
      }
     }
     else{
      while(gyro < angel){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_right();
      _delay(0.05);
      }
     }
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);   
     delay(250); 
    }
    if(side == 2){  
     dis_front_read();
     dis_left_b = IR(A15);
     dis_right_b = IR(A10);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     Serial.print("R:");Serial.println(dis_right);
     Serial.print("Rb:");Serial.println(dis_right_b);
     if(Global_hand == 0){
      diff= dis_left - dis_left_b;
      if(100-diff*75 > 0){
       Encoder_1.setMotorPwm(35-diff*75);
       Encoder_2.setMotorPwm(-35-diff*75);
      }
      else{
       Encoder_1.setMotorPwm(0);
       Encoder_2.setMotorPwm(-25);         
      }
      if((dis_right+dis_right_b)/2 > 12){
       Encoder_1.setMotorPwm(80);
       Encoder_2.setMotorPwm(-80*0.7);
      }
      else if((dis_right+dis_right_b)/2 < 7.5){
       Encoder_1.setMotorPwm(80*0.7);
       Encoder_2.setMotorPwm(-80);
      }
     }
     else{
      diff= dis_right - dis_right_b;
      if(80+diff*75 < 100){
       Encoder_1.setMotorPwm(80+diff*75);
       Encoder_2.setMotorPwm(-80+diff*75);
      }
      else{
       Encoder_1.setMotorPwm(70);
       Encoder_2.setMotorPwm(0);                 
      }
      if((dis_right+dis_right_b)/2 > 12){
       Encoder_1.setMotorPwm(80);
       Encoder_2.setMotorPwm(-80*0.7);
      }
      else if((dis_right+dis_right_b)/2 < 7.5){
       Encoder_1.setMotorPwm(80*0.7);
       Encoder_2.setMotorPwm(-80);
      }
     }
    }
}
void abyss(){
 HSV();
 if(BlackV > V){
    dis_front_read();
    dis_left =  IR(A14);
    dis_right =  IR(A9);
   examination();
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(0.9);       
   Encoder_1.setTarPWM(-80);
   Encoder_2.setTarPWM(80);
   _delay(0.5);           

   while (dis_left < 30 && dis_right < 30){
      Encoder_1.setTarPWM(-80);
      Encoder_2.setTarPWM(80);
   }
 }
 else if(H>BlueH){
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(5);     
   Encoder_1.setMotorPwm(50);
   Encoder_2.setMotorPwm(-50);     
   _delay(5);
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
 }
 /*int w = analogRead(A8);
  lcd.setCursor(7,1);
  lcd.print("W:");
  lcd.setCursor(9,1);
  lcd.print(w);
  if(w > 300){
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(0.5);       
   Encoder_1.setTarPWM(-100/100.0*255);
   Encoder_2.setTarPWM(100/100.0*255);
   _delay(0.5);           
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   gyro_rotate(0);      
   gyro_rotate(0);
  }*/
}
void complect(){
 Encoder_1.setTarPWM(0);
 Encoder_2.setTarPWM(0);
 _delay(0.5);
  servo1.write(1);
  delay(2000);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(15);
  delay(200);
  servo1.write(1);
  delay(200);
  servo1.write(168);
  delay(2000); 
  Encoder_1.setTarPWM(MAXSPEED/100.0*255);
  Encoder_2.setTarPWM(-MAXSPEED/100.0*255);
  _delay(1);
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(0.5);    
}
void rpi(){  //no working now on robofinist2022,because we haven't rpi zero
  if (Serial2.available()) {
    // get the new byte:
    char inChar = (char)Serial2.read();
    // add it to the inputString:
    //inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (symbol == 1) {
  symbol=0;
  char string = inputString.charAt(0);      
  inputString = "";
  stringComplete = false;    
  led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
  led_ring.show(); 
    lcd.setCursor(2, 1);
    lcd.print(string);
   if(string == 'H'){
      lcd.setCursor(2, 1);
      lcd.print("H    ");
       for(y = 1; y <= 3; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 255, 0);
         led_ring.show();
         delay(166);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }
      complect();
      complect();
      complect();     
     }
   else if(string == 'S'){
       lcd.setCursor(2, 1);
       lcd.print("S    ");
       for(y = 1; y <= 2; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 255, 255, 255);
         led_ring.show();
         delay(250);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }
    complect();
    complect(); 
    }
   else if(string == 'U'){
       lcd.setCursor(2, 1);
       lcd.print("U    ");
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 255, 255, 255);
         led_ring.show();
         delay(500);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       } 
   if(string == 'R'){
    lcd.setCursor(2, 1);
    lcd.print("Red");
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 255, 0, 0);
       led_ring.show();
       delay(50);
       }
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
     complect();
    }    
   if(string == 'G'){
    lcd.setCursor(2, 1);
    lcd.print("Green");
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 0, 255, 0);
       led_ring.show();
       delay(50);
       }
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
    }
   if(string == 'Y'){
    lcd.setCursor(2, 1);
    lcd.print("Yeallow");
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 255, 255, 0);
       led_ring.show();
       delay(50);
       }
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
    complect();
    }
   }
   if(inChar=='@'){
    symbol=1;
   }
  }
}
/*void alignment(){ // выравнивание
  dis_left =  IR(A14);
  dis_right =  IR(A9); 
  examination();
  if(dis_left <= 20 && dis_right <= 20){
    if(dis_left - dis_right > 5){while(dis_left - dis_right > 5){
     rotate_left();
     _delay(0.53);
     rotate_right();
     _delay(0.5);
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
    }}
    else if(dis_right - dis_left > 10){while(dis_right - dis_left > 10){
     rotate_right();
     _delay(0.5);
     rotate_left();
     _delay(0.5);
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
    }}
   }
   else{
    if(dis_left < 20){
     if(dis_left > 13){
      Encoder_1.setMotorPwm(-100);
      Encoder_2.setMotorPwm(-100);
      _delay(0.15); 
     }
     else if(dis_left < 10){
      Encoder_1.setMotorPwm(100);
      Encoder_2.setMotorPwm(100);
      _delay(0.15);       
     }
    }
    else if(dis_right < 20){
     if(dis_right > 13){
      Encoder_1.setMotorPwm(100);
      Encoder_2.setMotorPwm(100);
      _delay(0.15); 
     }
     else if(dis_right < 10){
      Encoder_1.setMotorPwm(-100);
      Encoder_2.setMotorPwm(-100);
      _delay(0.15); 
     }
    }
   }
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
    move(3,0);
    _delay(1);
}*/
int random_hand(){
  Random = analogRead(random_port);
  if(Random % 2 == 1){
    variant = 0;
    return 0;
  }
  else{
    variant = 1;
    return 1;
  }
}
void gyro_y(){
 gyro_0.update();
 int gyro_y = gyro_0.getAngle(1); 
 lcd.setCursor(11,0);
 lcd.print("Y:");
 lcd.setCursor(13,0);
 lcd.print(gyro_y);
 if(gyro_y < -40){
  Encoder_1.setMotorPwm(-120);
  Encoder_2.setMotorPwm(120);  
  _delay(0.14);
  Encoder_1.setMotorPwm(-120);
  Encoder_2.setMotorPwm(120);
  _delay(0.14);  
  Encoder_1.setTarPWM(30);
  Encoder_2.setTarPWM(-30);
  _delay(2); 
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(0.075);
  Encoder_1.setMotorPwm(-70);
  Encoder_2.setMotorPwm(70);     
  _delay(1); 
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  move(3,0);
  _delay(0.5);      
 }
 else if(gyro_y < -20 && gyro_y > -40){
  dis_left =  IR(A14);
  dis_right =  IR(A9); 
  while(dis_left <= 25 && dis_right <= 25){
   Encoder_1.setMotorPwm(120);
   Encoder_2.setMotorPwm(-120);  
   _delay(0.14);    
   dis_left =  IR(A14);
   dis_right =  IR(A9);   
  }
   Encoder_1.setMotorPwm(120);
   Encoder_2.setMotorPwm(-120);  
   _delay(0.14);
   Encoder_1.setMotorPwm(120);
   Encoder_2.setMotorPwm(-120);  
   _delay(0.14);  
   Encoder_1.setMotorPwm(0);
   Encoder_2.setMotorPwm(0);
   move(3,0);
   _delay(0.5);
 }
 else if(gyro_y > 20){
  Encoder_1.setMotorPwm(30);
  Encoder_2.setMotorPwm(-30);  
  while(dis_left <= 25 && dis_right <= 25){
   Encoder_1.setMotorPwm(30);
   Encoder_2.setMotorPwm(-30);
   dis_left =  IR(A14);
   dis_right =  IR(A9); 
  }
   Encoder_1.setMotorPwm(120);
   Encoder_2.setMotorPwm(-120);  
   _delay(0.14);
   Encoder_1.setMotorPwm(120);
   Encoder_2.setMotorPwm(-120);  
   _delay(0.14);  
   Encoder_1.setMotorPwm(0);
   Encoder_2.setMotorPwm(0);
   move(3,0);
   _delay(0.5);
 }
}
float dis_front_read(){
 VL53L0X_RangingMeasurementData_t measure;
 lox.rangingTest(&measure, false);
 dis_front = measure.RangeMilliMeter / 10;
 return dis_front;
}
void HSV(){
  ReadRGBSensor();
  RGBtoHSV((float) ColorRed, (float) ColorGreen, (float) ColorBlue, H, S, V); 
}
void ReadRGBSensor(){
  Wire.beginTransmission(htColorSensorAddress);
  Wire.write(0x43); // Red+Green+Blue
  Wire.endTransmission();

  //the code below requests 3 bytes of information RGB
  Wire.requestFrom(htColorSensorAddress, 3); 
   while(Wire.available() < 3);
   ColorRed = Wire.read();
   ColorGreen = Wire.read();
   ColorBlue = Wire.read();
}
void RGBtoHSV(float red, float green, float blue, float &hue, float &sat, float &value)
{
  hue = 0;
  sat = 0;
  value = 0;

  //   Value
  float rgb_max = max3(red, green, blue);
  float rgb_min;
  value = rgb_max / 2.56;
  if (value == 0){
    hue = -1;
    sat = -1;
    return;
  }

  //   Saturation
  red /= rgb_max;
  green /= rgb_max;
  blue /= rgb_max;

  rgb_max = max3(red, green, blue);
  rgb_min = min3(red, green, blue);
  sat = (rgb_max - rgb_min) * 100;
  if (sat == 0){
    hue = -1;
    return;
  }

  //   Hue

  red = (red - rgb_min) / (rgb_max - rgb_min);
  green = (green - rgb_min) / (rgb_max - rgb_min);
  blue = (blue - rgb_min) / (rgb_max - rgb_min);

  rgb_max = max3(red, green,blue);
  rgb_min = min3(red, green,blue);

  if (rgb_max == red){
    hue = 0.0 + 60.0*(green-blue);
    if (hue < 0.0){
      hue += 360.0;
    }
  } else if (rgb_max == green){
    hue = 120.0 + 60.0 * (blue-red);
  } else {
    hue = 240.0 + 60.0 * (red-green);
  }
}
void setup() {
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  float dis_left =  IR(A14);
  float dis_right =  IR(A9);
  //float dis_right1 =  IR(A10);
  servo1.attach(9);
  servo1.write(168);
  pinMode(13,OUTPUT);
  delay(2000);
  //Servo1.move(180,200,NULL);
  Wire.begin();
  Wire.beginTransmission(htColorSensorAddress); //open communication to sensor 
  Serial.begin(9600);
  Wire.write(0x00); //blank packet 
  //inputString.reserve(200);
  gyro_0.begin();
  #ifdef MeAuriga_H
    // 12 LED Ring controller is on Auriga D44/PWM
    led_ring.setpin( 44 );
  #endif
  lcd.init();                     
  lcd.backlight();// Включаем подсветку дисплея
  uint16_t result;
  float temp;
 // pinMode(trigPin, OUTPUT); // назначаем trigPin (Pin8), как выход
 // pinMode(echoPin, INPUT); // назначаем echoPin (Pin9), как вход
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
   variant = random_hand();
  if(variant == 0){
    Global_hand = 0;
    lcd.print("Left_hand");
  }
  else{
   Global_hand = 1;
    lcd.print("right_hand");
  }
  delay(5000);
 //int u = analogRead(A10);
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   /*_delay(0.5);       
   Encoder_1.setTarPWM(MAXSPEED/100.0*255);
   Encoder_2.setTarPWM(-MAXSPEED/100.0*255);
   _delay(0.25);           
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(0.5); */      
   while(1) {
    int duration, cm; // назначаем переменную "cm" и "duration" для показаний датчика
    //Serial.println (w);
    gyro_y(); 
    dis_front_read();
    abyss();
    int i;   
    Wire.beginTransmission(0x5A);
    Wire.write(0x07);            // sends instruction byte
    Wire.endTransmission(false);     // stop transmitting
    Wire.requestFrom(0x5A, 3);//Send data n-bytes read
    result = Wire.read(); //Receive DATA
    result |= Wire.read() << 8; //Receive DATA
    uint8_t pec = Wire.read();
    temp = result * 0.02 - 273.15;
    //Serial.println(temp);   
    w = analogRead(A10);
/*    if(w -u >=40){
     w = u;
    }*/
    Wire.beginTransmission(0x5A);
    Wire.write(0x07);            // sends instruction byte
    Wire.endTransmission(false);     // stop transmitting
    Wire.requestFrom(0x5A, 3);//Send data n-bytes read
    result = Wire.read(); //Receive DATA
    result |= Wire.read() << 8; //Receive DATA
    temp = result * 0.02 - 273.15;
 /*   lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.setCursor(2, 1);
    lcd.print(temp);
    lcd.setCursor(6, 1);
    lcd.print(" ");*/
    //Serial.println(temp);
      if(temp >= 30 && temp < 1000){
       for(y = 1; y <= 10; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 255, 0, 0);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }   
      complect();
     }
    gyro_y();
    int knop = digitalRead(knopka);
    Serial.print("Knopka: ");
    Serial.println(knop);
    if(knop == 0){
     variant = random_hand();
     if(variant == 0){
      Global_hand = 0;
      lcd.setCursor(0, 0);
      lcd.print("Left_hand ");
     }
     else{
      Global_hand = 1;
      lcd.setCursor(0, 0);
      lcd.print("right_hand");
      }
       move(3,0);
       _delay(0.5);
       for(y = 1; y <= 10; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 0, 255);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }         
      knop = digitalRead(knopka);
      while(knop == 1){
        knop = digitalRead(knopka);
        delay(1);
        led_ring.setColor( RINGALLLEDS, 0, 0, 255 );
        led_ring.show();
      }
      led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
      led_ring.show();
    }
    abyss();
   /* if(stringComplete){
     rpi();
    }*/
    dis_front_read();
    dis_left =  IR(A14);
    dis_right =  IR(A9);
    dis_left_b = IR(A15);
    dis_right_b = IR(A10);
    Serial.print("L: ");
    Serial.println(dis_left);
    Serial.print("R: ");
    Serial.println(dis_right);
    Serial.print("F: ");
    Serial.println(dis_front);
    Serial.print("L2: ");
    Serial.println(dis_left_b);
     Serial.print("R2: ");
    Serial.println(dis_right_b);
    examination();
    if(dis_left > 25 && dis_right > 25 && dis_left_b > 25 && dis_right_b > 25){
     variant = random_hand();
     if(variant == 0){
      gyro_rotate(0);
     }
     else{
      gyro_rotate(1);
     }
    }
    else{
      if(Global_hand == 0){
       if(dis_left > 25 ){
         if(dis_left_b > 25) gyro_rotate(0);
         else{
          Encoder_1.setTarPWM(80);
          Encoder_2.setTarPWM(-80);          
         }
        }
        else if(dis_front < 20){
         gyro_rotate(1);
        }
        else{
         gyro_rotate(2);
        }
       }
       else{
        if(dis_right > 25){
         if(dis_right_b > 25) gyro_rotate(1);
         else{
          Encoder_1.setTarPWM(80);
          Encoder_2.setTarPWM(-80);          
         }
        }
        else if(dis_front < 20){
         gyro_rotate(0);
        }
        else{
         gyro_rotate(2);
        }      
       }
      }  
     gyro_y();
      _loop();
  }
}
void loop() {
  _loop(); 
}
