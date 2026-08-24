#include <Wire.h>
#include <math.h>
int valr=0;
int valrb=0;
int vall=0;
int vallb=0;
int naprr=1;
int naprrb=1;
int naprl=1;
int naprlb=1;
int num;
char c;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);                
  Wire.onReceive(receiveEvent);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A7,OUTPUT);  
  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5,max(min(valr,254),0));
  digitalWrite(8,1-naprr);
  digitalWrite(2,naprr);
  analogWrite(3,max(min(valrb,254),0));
  digitalWrite(7,naprrb);
  digitalWrite(1,naprrb);
  analogWrite(9,max(min(vallb,254),0));
  digitalWrite(0,1-naprlb);
  digitalWrite(A7,1-naprlb);
  analogWrite(11,max(min(vall,254),0));
  digitalWrite(A1,naprl);
  digitalWrite(A2,1-naprl);
}
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    c = Wire.read(); // receive byte as a character
    if(c=='l') vall=Wire.read();
    else if(c=='r') valr=Wire.read();
    else if(c=='b'){
      c=Wire.read();
      if(c=='l') vallb=Wire.read();
      else if(c=='r') valrb=Wire.read();  
    }
    else if(c=='q'){
      c=Wire.read();
      if(c=='l')naprl=abs(naprl-1);
      else if(c=='r')naprr=abs(naprr-1);
      else if(c=='b'){
        c=Wire.read();
        if(c=='l')naprlb=abs(naprlb-1);
        else if(c=='r')naprrb=abs(naprrb-1);
      }
    }
  }
}
