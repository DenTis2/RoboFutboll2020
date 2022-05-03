 int D;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 pinMode(2,INPUT);
 pinMode(3,INPUT);
 pinMode(4,INPUT);
 pinMode(5,INPUT);
 pinMode(6,INPUT);
 pinMode(7,INPUT);
 pinMode(8,INPUT);
 pinMode(9,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 int coff = 200;
 int button1_camera_up = digitalRead(5);
 int button2_camera_centre = digitalRead(4);
 int button3_camera_dn = digitalRead(2);
 int button4_manipulator_shvat = digitalRead(6);
 int button5_manipulanor_rashvat = digitalRead(2);
 int button6_power_plus = digitalRead(9);
 int button7_power_standart = digitalRead(7);
 int button8_power_minus = digitalRead(8);
 if(button4_manipulator_shvat == 1){
  Serial.print("E");
  Serial.println(80);
  while(button4_manipulator_shvat == 1){
  delay(10);
  button4_manipulator_shvat = digitalRead(5);
  }
 }
 if(button5_manipulanor_rashvat == 1){
  Serial.print("E");
  Serial.println(-80);
  while(button5_manipulanor_rashvat == 1){
  delay(10);
  button5_manipulanor_rashvat = digitalRead(6);
  }
 }
 if(button2_camera_centre == 1){
  D = 90;
 }
 if(button1_camera_up == 1){
  D = D + 2;
  delay(100);
 }
 if(button3_camera_dn == 1){
  D = D - 2;
  delay(100);
 }
 if( D > 175){
  D = 175;
 }
 if( D < 5){
  D = 5;
 }
 Serial.print("D");
 Serial.println(D);
 if (button7_power_standart == 1){
  coff = 200;
 }
 if(button6_power_plus == 1){
  coff = coff + 50;
  while(button8_power_minus == 1){
  delay(10);
  button6_power_plus = digitalRead(7);
  }
 }
 if(button8_power_minus == 1){
  coff = coff - 50;
  while(button8_power_minus == 1){
  delay(10);
  button8_power_minus = digitalRead(9);
  }
 }
 if (coff > 300){
  coff = 300;
 }
 if(coff < 0){
  coff = 0;
 }
 int joy_UpDn = analogRead(A0); 
 int joy_LeftRight = analogRead(A2);
 int joy_ForwardBack = analogRead(A1);
 int joy1 = map(joy_UpDn,135,920,1100 + coff,1900 - coff);
 int joy2 = map(joy_ForwardBack,0,910,1100 + coff,1900 - coff);
 int joy3 = map(joy_LeftRight,0,910,1100 + coff,1900 - coff);
 /*Serial.print(joy_UpDn);
 Serial.print(" ");
 Serial.print(joy_ForwardBack);
 Serial.print(" ");
 Serial.println(joy_LeftRight);*/
 Serial.print("C");
 if(joy1<1550 && joy1>1450){
 Serial.println(1500);
 }else{
 Serial.println(joy1);
 }
 int motor_a = joy2 - joy3 + 1500;
 int motor_b = joy2 + joy3 - 1500;
 if (motor_b > 1900){
  motor_b = 1900;
 }
 if(motor_a > 1900){
  motor_a = 1900;
 }
 if (motor_b < 1100){
  motor_b = 1100;
 }
 if(motor_a < 1100){
  motor_a = 1100;
 }
 Serial.print("A");
 if(motor_a<1525 && motor_a>1475){
 Serial.println(1500);
 }else{
 Serial.println(motor_a);
 } 
 Serial.print("B");
 if(motor_b<1525 && motor_b>1475){
 Serial.println(1500);
 }else{
 Serial.println(motor_b);
 } 
 delay(100);
}
