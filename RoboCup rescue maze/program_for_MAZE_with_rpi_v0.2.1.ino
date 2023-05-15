#include <Arduino.h>                                             
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // Устанавливаем дисплей
#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0
#define MAXSPEED 50
//Servo servo1;
//String inputString = "";
int random_port = A7;
int variant;
int Random;
int Global_hand;
int t = 5;
int y;
int i;
int w;
int storona;
int knopka = A13;
int angel;
int gyro;
int lastgyro;
int trigPin = 60; // назначаем имя для Pin8
int echoPin = 65; // назначаем имя для Pin9
int ultra9;
float dis_front;
float dis_left;
float dis_right;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
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
 move(3, 50/ 100.0 *255);
}
void rotate_right(){
 move(4, 50 / 100.0 * 255);
}
void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

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
 if(dis_front < 0 || dis_front > 50){
  dis_front = 50;
 }
 if(dis_left < 0 || dis_left > 50){
  dis_left = 50;
 }
 if(dis_right < 0 || dis_right > 50){
  dis_right = 50;
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
  }else if(direction == 3){
    leftSpeed = -speed;
    rightSpeed = -speed;
  }else if(direction == 4){
    leftSpeed = speed;
    rightSpeed = speed;
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
     dis_front =  IR(A15);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     if(dis_right <= 20){
      storona = 1;
     }
     else{
      storona = 0;
     }
     if(dis_front < 25){
      Encoder_1.setTarPWM(64);
      Encoder_2.setTarPWM(-64);
      _delay(2); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(-135);
      Encoder_2.setMotorPwm(135);     
      _delay(1); 
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);      
       move(3,0);
       _delay(0.5);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     if(gyro > -90){
      angel = lastgyro - 90;
      while(angel - gyro < 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro - 90;
      while(gyro > -180 && -90 >= gyro|| gyro > angel + 360){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);        
      }
     }
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);  
     _delay(0.2);
     gyro_0.update();
     gyro = gyro_0.getAngle(3);
     if(lastgyro > -90){
      while(angel-gyro > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.0075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else if(180 >= gyro && gyro >= 90 ){
      while(180 - gyro > 10){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.0075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
     if(storona == 1){
      Encoder_1.setTarPWM(-64);
      Encoder_2.setTarPWM(64);
      _delay(2); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(135);
      Encoder_2.setMotorPwm(-135);     
      _delay(1); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      move(3,0);
      _delay(0.5);        
      storona = 0;
     }
 /*    gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     angel = lastgyro - 37.5;
     if(angel < -180){
      while(gyro > -170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      rotate_left();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      }
      gyro_0.update();
      while(gyro > 180 + (angel + 180) || gyro > -180 && gyro < -170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      rotate_left();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      }
     }
     else{
      led_ring.setColor( 6, 0, 0, 100);
      led_ring.show();
      while(gyro > angel){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);     
      rotate_left();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      }
     }*/
    move(3,0);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(0.5);
     Encoder_1.setTarPWM(-1 * -MAXSPEED/100.0*255);
     Encoder_2.setTarPWM(-MAXSPEED/100.0*255);
     _delay(0.55);
    move(3,0);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);   
    delay(250);    
  }
  if(side == 1){
     led_ring.setColor( 9, 100, 0, 0);
     led_ring.show();
     dis_front =  IR(A15);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     if(dis_left <= 20){
      storona = 1;
     }
     else{
      storona = 0;
     }
     if(dis_front < 25){
      Encoder_1.setTarPWM(64);
      Encoder_2.setTarPWM(-64);
      _delay(2); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(-135);
      Encoder_2.setMotorPwm(135);     
      _delay(1); 
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);      
       move(3,0);
       _delay(0.5);
     }
     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     if(gyro < 90){
      angel = lastgyro + 90;
      while(angel - gyro > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro + 90;
      while(gyro < 180 && 90 <= gyro|| gyro < angel - 360){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);        
      }
     }
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);  
     _delay(0.2);
     gyro_0.update();
     gyro = gyro_0.getAngle(3);
     if(lastgyro < 90){
      while(angel-gyro < 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.0075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else if(-180 <= gyro && gyro < -90 ){
      while(180 + gyro > 10){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.0075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);        
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
    if(storona == 1){
      Encoder_1.setTarPWM(-64);
      Encoder_2.setTarPWM(64);
      _delay(2); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(135);
      Encoder_2.setMotorPwm(-135);     
      _delay(1); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      move(3,0);
      _delay(0.5);        
      storona = 0;
     }
/*     gyro_0.update();
     lastgyro = gyro_0.getAngle(3);
     gyro = lastgyro;
     angel = lastgyro + 37.5;
     if(angel > 180){
      while(gyro < 170){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      rotate_right();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      gyro_0.update();
      }
      while(gyro < -180 + (angel - 180) || gyro > 170 && gyro < 180){
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      }
     }
     else{
      led_ring.setColor( 9, 0, 0, 100);
      led_ring.show();
      while(gyro < angel){
      gyro_0.update();
      gyro = gyro_0.getAngle(3);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);      
      rotate_right();
      _delay(0.075);
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      }
     }
 */
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.5); 
     Encoder_1.setTarPWM(MAXSPEED/100.0*255);
     Encoder_2.setTarPWM(-MAXSPEED/100.0*255);
     _delay(0.5); 
     move(3,0);
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);   
     delay(250); 
    }
    if(side == 2){
       
     Encoder_1.setMotorPwm(360);
     Encoder_2.setMotorPwm(-360);  
     _delay(0.14);
     Encoder_1.setMotorPwm(360);
     Encoder_2.setMotorPwm(-360);
     _delay(0.14);
     Encoder_1.setMotorPwm(360);
     Encoder_2.setMotorPwm(-360);     
     _delay(0.14);
     Encoder_1.setMotorPwm(277.5 + t);
     Encoder_2.setMotorPwm(-360);
     _delay(1); 
     dis_front =  IR(A15);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     alignment();
     /*dis_front =  IR(A9);
     if(dis_front > 13 && dis_front < 30){
      Encoder_1.setMotorPwm(100);
      Encoder_2.setMotorPwm(100);
      _delay(0.15);     
     }
     else if(y < 11){
      Encoder_1.setMotorPwm(-100);
      Encoder_2.setMotorPwm(-100);
      _delay(0.15);           
     }*/
    }
}
void abyss(){
 int w = analogRead(A8);
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
  }
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
void rpi(){  //no working now on robofinist,because we havent rpi zero
  char string = inputString.charAt(0);      
  inputString = "";
  stringComplete = false;    
  int y;
  int i;
  led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
  led_ring.show(); 
   if(string == 'H'){
       for(y = 1; y <= 10; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 255, 0);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }      
      complect();
      complect();
      complect();     
     }
   else if(string == 'S'){
       for(y = 1; y <= 10; y++){
        for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
        {
         led_ring.setColor( i, 0, 255, 0);
         led_ring.show();
         delay(50);
        }
        led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
        led_ring.show();
       }
    complect();
    complect(); 
    }
   else if(string == 'U'){
       for(y = 1; y <= 10; y++){
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
   if(string == 'R'){
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 255, 0, 0);
       led_ring.show();
       }
       delay(1000);
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
    //complect();
    }    
   if(string == 'G'){
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 0, 255, 0);
       led_ring.show();
       }
       delay(1000);
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
    }
   if(string == 'Y'){
    for(int p = 1; y <= 10; y++){
     for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
      {
       led_ring.setColor( i, 255, 255, 0);
       led_ring.show();
       }
       delay(1000);
       led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
       led_ring.show();
      }   
    //complect();
    }
}
void alignment(){ // выравнивание
  dis_front =  IR(A15);
  dis_left =  IR(A14);
  dis_right =  IR(A9); 
  examination();
  if(dis_left <= 20 && dis_right <= 20){
    if(dis_left - dis_right > 5){while(dis_left - dis_right > 5){
     rotate_left();
     _delay(0.53);
     rotate_right();
     _delay(0.5);
     dis_front =  IR(A15);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
    }}
    else if(dis_right - dis_left > 10){while(dis_right - dis_left > 10){
     rotate_right();
     _delay(0.5);
     rotate_left();
     _delay(0.5);
     dis_front =  IR(A15);
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
}
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
  Encoder_1.setMotorPwm(-360);
  Encoder_2.setMotorPwm(360);  
  _delay(0.14);
  Encoder_1.setMotorPwm(-360);
  Encoder_2.setMotorPwm(360);
  _delay(0.14);  
  Encoder_1.setTarPWM(64);
  Encoder_2.setTarPWM(-64);
  _delay(2); 
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  _delay(0.075);
  Encoder_1.setMotorPwm(-135);
  Encoder_2.setMotorPwm(135);     
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
   Encoder_1.setMotorPwm(360);
   Encoder_2.setMotorPwm(-360);  
   _delay(0.14);    
   dis_left =  IR(A14);
   dis_right =  IR(A9);   
  }
   Encoder_1.setMotorPwm(360);
   Encoder_2.setMotorPwm(-360);  
   _delay(0.14);
   Encoder_1.setMotorPwm(360);
   Encoder_2.setMotorPwm(-360);  
   _delay(0.14);  
   Encoder_1.setMotorPwm(0);
   Encoder_2.setMotorPwm(0);
   move(3,0);
   _delay(0.5);
 }
 else if(gyro_y > 20){
  Encoder_1.setMotorPwm(64);
  Encoder_2.setMotorPwm(-64);  
  while(dis_left <= 25 && dis_right <= 25){
   Encoder_1.setMotorPwm(64);
   Encoder_2.setMotorPwm(-64);
   dis_left =  IR(A14);
   dis_right =  IR(A9); 
  }
   Encoder_1.setMotorPwm(360);
   Encoder_2.setMotorPwm(-360);  
   _delay(0.14);
   Encoder_1.setMotorPwm(360);
   Encoder_2.setMotorPwm(-360);  
   _delay(0.14);  
   Encoder_1.setMotorPwm(0);
   Encoder_2.setMotorPwm(0);
   move(3,0);
   _delay(0.5);
 }
}
void setup() {
  float dis_front =  IR(A15);
  float dis_left =  IR(A14);
  float dis_right =  IR(A9);
  servo1.attach(9);
  servo1.write(168);
  pinMode(13,OUTPUT);
  delay(2000);
  //Servo1.move(180,200,NULL);
  Wire.begin();
  Serial.begin(9600); 
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
  pinMode(trigPin, OUTPUT); // назначаем trigPin (Pin8), как выход
  pinMode(echoPin, INPUT); // назначаем echoPin (Pin9), как вход
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
    /*if(abs(w - u >=15)){
     u = w;
    }
    else if( abs (w - u) >= 5 ){
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(0.5);       
    Encoder_1.setTarPWM(-100/100.0*255);
    Encoder_2.setTarPWM(100/100.0*255);
    _delay(0.5);           
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     move(3, 80 / 100.0 * 255);
     _delay(1.25);
     move(3, 0);
     Encoder_1.setTarPWM(-1 * -75/100.0*255);
     Encoder_2.setTarPWM(-75/100.0*255);
     _delay(1);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);  
    _delay(0.5);      
    }*/
    /*if(stringComplete == true){
     rpi();
    }*/
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
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.setCursor(2, 1);
    lcd.print(temp);
    lcd.setCursor(6, 1);
    lcd.print(" ");
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
 /*   else if( w - u >= 5 ){
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(0.5);       
    Encoder_1.setTarPWM(-100/100.0*255);
    Encoder_2.setTarPWM(100/100.0*255);
    _delay(0.5);           
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     move(3, 80 / 100.0 * 255);
     _delay(1.25);
     move(3, 0);
     Encoder_1.setTarPWM(-75/100.0*255);
     Encoder_2.setTarPWM(-75/100.0*255);
     _delay(1);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);  
    _delay(0.5);   
    cm = 24.97684478;
    }
    else if(u - w >= 3){
     u= w;
    }*/
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
    dis_front =  IR(A15);
    dis_left =  IR(A14);
    dis_right =  IR(A9);
    examination();
    if(dis_left > 25 && dis_right > 25){
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
       if(dis_left > 25){
         gyro_rotate(0);
        }
        else if(dis_front < 15){
         gyro_rotate(1);
        }
        else{
         gyro_rotate(2);
        }
       }
       else{
        if(dis_right > 25){
         gyro_rotate(1);
        }
        else if(dis_front < 15){
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
