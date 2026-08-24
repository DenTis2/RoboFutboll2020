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
#define MAXSPEED 55
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
long  Encoder1, Encoder2;
float current_1, current_2;
int stena=25;
bool black_pol=1;
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
 move(3, 70/ 100.0 *255);
}
void rotate_right(){
 move(4, 70 / 100.0 * 255);
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
struct myNode{
uint8_t wall[4]={0,0,0,0};
//char tile=' ';
//bool victim=0;
uint8_t count=0; 
//int coordinata_x, coordinata_y; 
};
myNode mymap[30][30];


int cord_x=15, cord_y=25;
int napravlenie=2;

uint8_t tam_l,tam_r;
uint8_t x_left, x_front, x_right, x _back, y_left, y_front, y_right, y_back;

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
  if(direction == 1){ //nazad
    leftSpeed = -speed;
    rightSpeed = speed;
  }else if(direction == 2){ //vpered
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
     led_ring.setColor( 12, 0, 0, 100);
     led_ring.show();
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.075);
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     if(dis_right <= stena){
      storona = 1;
     }
     else{
      storona = 0;
     }
     if(dis_front < stena){
      Encoder_1.setTarPWM(135);
      Encoder_2.setTarPWM(-135);
      _delay(1); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(-64);
      Encoder_2.setMotorPwm(64);     
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
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro - 90;
      while(gyro > -180 && -90 >= gyro || gyro > angel + 360){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);  
       _delay(0.01);
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
      while(abs(gyro)-abs(angel) > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);   
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else if(180 >= gyro && gyro >= 90 ){
      while(360 + angel - gyro > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0); 
       _delay(0.01);   
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
     move(1,100);
     _delay(0.1);
     move(1,0);
     if(storona == 1){
      Encoder_1.setTarPWM(-135); //nazad
      Encoder_2.setTarPWM(135);
      _delay(1); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);        
      storona = 0;
     }

    alignment();  
    napravlenie--;
    if(napravlenie==0) napravlenie=4;
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    _delay(0.5); 
    dis_front_read();
    if(dis_front>=stena) gyro_rotate(2);
     _delay(0.25);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);   
    delay(250);    

    led_ring.setColor( 12, 0, 0, 0 );
    led_ring.show();
  }
  
  if(side == 1){
     led_ring.setColor( 6, 0, 0, 100);
     led_ring.show();
     dis_front_read();
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     if(dis_left <= 25){
      storona = 1;
     }
     else{
      storona = 0;
     }
     if(dis_front < 15){
      Encoder_1.setTarPWM(135);
      Encoder_2.setTarPWM(-135);
      _delay(1); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);
      Encoder_1.setMotorPwm(-64);
      Encoder_2.setMotorPwm(64);     
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
       _delay(0.01); 
       gyro_0.update();
       gyro = gyro_0.getAngle(3);
      }
     }
     else{
       angel = lastgyro + 90;
      while(gyro < 180 && 90 <= gyro || gyro < angel - 360){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_right();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);  
       _delay(0.01);
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
      while(gyro - angel > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0); 
       _delay(0.01);       
       gyro_0.update();
       gyro = gyro_0.getAngle(3);    
      }
     }
     else {
      while(360 - angel + gyro > 0){
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);     
       rotate_left();
       _delay(0.075);
       Encoder_1.setTarPWM(0);
       Encoder_2.setTarPWM(0);
       _delay(0.01);
       gyro_0.update();
       gyro = gyro_0.getAngle(3);          
      }
     }
    move(1,100);
     _delay(0.1);
     move(1,0);
    if(storona == 1){
      Encoder_1.setTarPWM(-135); //nazad
      Encoder_2.setTarPWM(135);
      _delay(2); 
      Encoder_1.setTarPWM(0);
      Encoder_2.setTarPWM(0);
      _delay(0.075);      
      storona = 0;
     }

     alignment();  
     napravlenie++;
     if(napravlenie==5) napravlenie=1;
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);
     _delay(0.5); 
     dis_front_read();
     if(dis_front>=stena) gyro_rotate(2);
     Encoder_1.setTarPWM(0);
     Encoder_2.setTarPWM(0);   
     delay(250); 

     led_ring.setColor( 6, 0, 0, 0 );
     led_ring.show();

    }
    
    if(side == 2){  
    dis_front_read();
    Encoder1=Encoder_1.getCurPos();
    Encoder2=Encoder_2.getCurPos();

    if(dis_front>=stena)
    while(abs(Encoder_1.getCurPos()-Encoder1)<470 && abs(Encoder_2.getCurPos()-Encoder2)<470){
     Encoder_1.setMotorPwm(180);
     Encoder_2.setMotorPwm(-180); 
     HSV();
     
     if(BlackV > V){
        abyss();
        black_pol=0;
        break;
     }
     _delay(0.01); 
     printl();
    }
     _delay(2);

     dis_front =  IR(A15);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     examination();
     alignment();    

  if(black_pol){
     if(napravlenie==1) cord_x--;
     else if(napravlenie==2) cord_y--;
     else if(napravlenie==3) cord_x++;
     else cord_y++;
    }
    black_pol=1;
    abyss();
   }
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
void abyss(){
  HSV();
 if(BlackV > V){
    Encoder1=Encoder_1.getCurPos();
    Encoder2=Encoder_2.getCurPos();

    while(abs(Encoder_1.getCurPos()-Encoder1)<470 && abs(Encoder_2.getCurPos()-Encoder2)<470){
     move(1,180);
     _delay(0.14); 
    }
    move(1,0);
     _delay(2);
     
    dis_left =  IR(A14);
    dis_right =  IR(A9);
    if(dis_left > dis_right){
       gyro_rotate(0);
      dis_front_read();
      if(dis_front<stena) gyro_rotate(0);
    }
    else{
       gyro_rotate(1);
      dis_front_read();
      if(dis_front<stena) gyro_rotate(1);
    }

 }
 else if(H>BlueH){
   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
   _delay(5);     
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
void rpi(){  //no working now on robofinist2022,because we haven't rpi zero
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
  dis_left =  IR(A14);
  dis_right =  IR(A9); 
  examination();

  if(dis_right<=stena && dis_left<=stena){
    while(dis_right-dis_left >2){
      rotate_right();
      _delay(0.2);  
      dis_front_read();
      //if(dis_front>=stena){move(2,180); _delay(0.02);}
      move(2, 180); _delay(0.002);
      rotate_left();
      _delay(0.1);
     dis_left =  IR(A14);
     dis_right =  IR(A9);
     printl();
    }
  }
    move(3,0);

    dis_right_b = IR(A10);
    dis_right =  IR(A9);
  if(dis_right_b<=stena && dis_right<=stena)
    while(dis_right-dis_right_b >2){
      rotate_right();
      _delay(0.2);
      dis_front_read();
      //if(dis_front>=stena){move(2,180); _delay(0.02);}
      move(2, 180); _delay(0.002);
      rotate_left();
      _delay(0.1);
     dis_right_b = IR(A10);
     dis_right =  IR(A9);
     printl();
    }

  dis_left = IR(A14);
  dis_left_b = IR(A15);
  if(dis_left<=stena && dis_left_b<=stena)
    while(dis_left_b-dis_left >2){
      rotate_right();
      _delay(0.2);
      dis_front_read();
      //if(dis_front>=stena){move(2,180); _delay(0.02);}
      move(2, 180); _delay(0.002);
      rotate_left();
      _delay(0.1);
      dis_left = IR(A14);
      dis_left_b = IR(A15);
      printl();
    }

  dis_right =  IR(A9); 
    if (dis_right<=5)
      while(dis_right<7){
        rotate_left();
       _delay(0.2);
      dis_front_read();
      //if(dis_front>=stena){move(2,180); _delay(0.02);}
      move(2, 180); _delay(0.002);
       rotate_left();
       _delay(0.1);
      dis_right =  IR(A9);
       printl();
      }

     dis_left =  IR(A14);
     if(dis_left<=5)
      while(dis_left<7){
        rotate_right();
       _delay(0.2);
      dis_front_read();
      //if(dis_front>=stena){move(2,180); _delay(0.02);}
      move(2, 180); _delay(0.002);
        rotate_left();
       _delay(0.1);
       dis_left = IR(A14);
       dis_left_b = IR(A15);
       printl();
    }

    move(2,180);
    _delay(0.05);
    move(2,0);
     
}

int random_hand(){
  // LEFT_HAND!!!################################################################################################################################################################################
  Random = analogRead(random_port);
  /*if(Random % 2 == 1){
    variant = 0;
    return 0;
  }
  else{
    variant = 1;
    return 1;
  }*/
  variant = 0;
  return 0;
}

void gyro_y(){
 gyro_0.update();
 int gyro_y = gyro_0.getAngle(1); 
 /*lcd.setCursor(11,0);
 lcd.print("Y:");
 lcd.setCursor(13,0);
 lcd.print(gyro_y);*/
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
void antizalip(){
  current_1=Encoder_1.getCurrentSpeed();
  current_2=Encoder_2.getCurrentSpeed();

  if(current_1==0 && current_2==0){
    move(1,180);
    _delay(0.02);
    move(1,0);
  }
}
void okruga(){
    dis_front_read();
    dis_left =  IR(A14);
    dis_right =  IR(A9);
    dis_left_b = IR(A15);
    dis_right_b = IR(A10);
    examination();    
    mymap[cord_x][cord_y].count+=1;

    if(dis_front<=stena) mymap[cord_x][cord_y].wall[napravlenie-1]=1;    
    if(dis_left<=stena && dis_left_b<=stena) mymap[cord_x][cord_y].wall[tam_l-1]=1;
    if(dis_right<=stena && dis_right_b<=stena) mymap[cord_x][cord_y].wall[tam_r-1]=1;

}
void printl(){
       lcd.setCursor(0,0);
     lcd.print("L:");
     lcd.print(IR(A14));
     lcd.setCursor(9,0);
     lcd.print("R:");
     lcd.print(IR(A9));
     
     /*lcd.setCursor(0,1);
     lcd.print("L:");
     lcd.print(IR(A15));
     lcd.setCursor(9,1);
     lcd.print("R:");
     lcd.print(IR(A10));*/

}
float dis_front_read(){
 VL53L0X_RangingMeasurementData_t measure;
 lox.rangingTest(&measure, false);
 dis_front = measure.RangeMilliMeter / 10;
 return dis_front;
}
void setup() {
  
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  servo1.attach(9);
  servo1.write(168);
  pinMode(13,OUTPUT);
  delay(2000);
  Wire.begin();
  Serial.begin(9600); 
  gyro_0.begin();
  #ifdef MeAuriga_H
    led_ring.setpin( 44 );
  #endif
  lcd.init();                     
  lcd.backlight();// Включаем подсветку дисплея
  uint16_t result;
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
  //ONLY LEFT HAND! ########################################################################################################################################################################
  //if(variant == 0){
  //  Global_hand = 0;
  //  lcd.print("Left_hand");
 // }
 // else{
 //  Global_hand = 1;
 //   lcd.print("right_hand");
 // }
 //######################################################################################################################################################################################### 
  delay(5000);

//  mymap[cord_x][cord_y].tile = 'S';

   Encoder_1.setTarPWM(0);
   Encoder_2.setTarPWM(0);
     
   while(1) {
    lcd.clear();
     printl();
     lcd.setCursor(0,1);
     lcd.print(cord_x);
     lcd.print(":");
     lcd.print(cord_y);
     lcd.print("   ");
     lcd.print(mymap[cord_x][cord_y].wall[0]);
     lcd.print(mymap[cord_x][cord_y].wall[1]);
     lcd.print(mymap[cord_x][cord_y].wall[2]);
     lcd.print(mymap[cord_x][cord_y].wall[3]);
     lcd.print(' ');
     lcd.print(mymap[cord_x][cord_y].count);
  
     
    int duration, cm; // назначаем переменную "cm" и "duration" для показаний датчика
    gyro_y(); 
  
    Serial.print("DDIIISSS:::   "); Serial.print(dis_front);
     
    Wire.beginTransmission(0x5A);
    Wire.write(0x07);            // sends instruction byte
    Wire.endTransmission(false);     // stop transmitting
    Wire.requestFrom(0x5A, 3);//Send data n-bytes read
    result = Wire.read(); //Receive DATA
    result |= Wire.read() << 8; //Receive DATA
    uint8_t pec = Wire.read();
    //Serial.println();   
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
   /* if(stringComplete){
     rpi();
    }*/

    dis_front_read();
    dis_left =  IR(A14);
    dis_right =  IR(A9);
    dis_left_b = IR(A15);
    dis_right_b = IR(A10);
    examination();

    tam_l=napravlenie-1, tam_r=napravlenie+1;
    if(tam_l==0) tam_l=4;
    if(tam_r==5) tam_r=1;
    okruga();

   if(mymap[cord_x][cord_y].count>1){
    if(mymap[cord_x][cord_y].wall[tam_r-1]==0){
      if(dis_right_b>=stena || abs(dis_right-dis_right_b) <5) gyro_rotate(1);
      else{
         while(dis_right_b<stena && dis_front>=stena){
          move(2,100);
          _delay(0.01);
          move(2,0);
          _delay(0.01);
          dis_right_b = IR(A10);
          dis_front_read();
          printl();
          antizalip();
         }
         move(2,0);
         gyro_rotate(0);
      }
    }
    else if(mymap[cord_x][cord_y].wall[tam_l-1]%2==0){
      if(dis_left_b>=stena) gyro_rotate(0);
      else{
        while(dis_left_b<stena && dis_front>=stena){
          move(2,100);
          _delay(0.01);
          move(2,0);
          _delay(0.01);
          dis_left_b = IR(A15);
          dis_front_read();
          printl();
          antizalip();
        }
        move(2,0);
        gyro_rotate(0);
      }
    }
    else gyro_rotate(2);
   }
   
   else if(dis_front>=stena) gyro_rotate(2);
   else if(dis_left>=stena){
      if(dis_left_b>=stena) gyro_rotate(0);
      else{
        while(dis_left_b<stena && dis_front>=stena){
          move(2,100);
          _delay(0.01);
          move(2,0);
          _delay(0.01);
          dis_left_b = IR(A15);
          dis_front_read();
          printl();
          antizalip();
        }
        move(2,0);
        gyro_rotate(0);
      }
   }
   else if (dis_right>=stena) {
      if(dis_right_b>=stena || abs(dis_right-dis_right_b) <5) gyro_rotate(1);
      else{
         while(dis_right_b<stena && dis_front>=stena){
          move(2,100);
          _delay(0.01);
          move(2,0);
          _delay(0.01);
          dis_right_b = IR(A10);
          dis_front_read();
          printl();
          antizalip();
         }
         move(2,0);
         gyro_rotate(0);
      }
   }
   else{
    gyro_rotate(0);
    gyro_rotate(0);
   }
     
    
     lcd.clear();
     gyro_y();
      _loop();
  }
}
void loop() {
  _loop(); 
}
