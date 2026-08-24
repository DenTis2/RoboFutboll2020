#include <AmperkaServo.h>
AmperkaServo serv;
int coun=0;
void setup() {
  // put your setup code here, to run once:
  serv.attach(22);
}

void loop() {
  //!Выгружает компект за 2 тика!
  if(coun<5)serv.writeSpeed(20);
  else if(coun<10)  serv.writeSpeed(-39.5);
  else coun=-1;
  delay(200);
  serv.writeSpeed(0);
  delay(1000);
  coun++;
}
