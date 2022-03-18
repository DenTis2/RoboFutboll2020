 #include <MeAuriga.h>
#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0
#include <Wire.h>

#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring( 0, 12 );
#endif

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  #ifdef MeAuriga_H
    // 12 LED Ring controller is on Auriga D44/PWM
    led_ring.setpin( 44 );
  #endif
 
}
 
uint16_t result;
float temp;
 
void loop() {
  int i;   
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(0x5A);
  Wire.write(0x07);            // sends instruction byte
  Wire.endTransmission(false);     // stop transmitting
 
  Wire.requestFrom(0x5A, 3);//Send data n-bytes read
  result = Wire.read(); //Receive DATA
  result |= Wire.read() << 8; //Receive DATA
 
  uint8_t pec = Wire.read();
   
  temp =  result*0.02-273.15;//

  if(temp >= 27){
   // led_ring.setColor( RINGALLLEDS, 50, 0, 0 );
    //led_ring.show();
    for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
    {
        // Turn on  LEDs one at a time with some interesting color
        Serial.print(i);
        led_ring.setColor( i, 100, 0, 0);
        led_ring.show();
        delay( 50 );
    }
    delay( 500 );
    led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
    led_ring.show();
  }
 
  Serial.print(temp);
 
  Serial.println();
  // delay(500)
    
    
    
    led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
    led_ring.show();
    delay( 500 );
    /*for ( i = 1; i <= AURIGARINGLEDNUM; i++ )
    {
        // Turn on  LEDs one at a time with some interesting color
        Serial.print(i);
        led_ring.setColor( i, 40, 10, 40);
        led_ring.show();
        delay( 200 );
    }*/
}
