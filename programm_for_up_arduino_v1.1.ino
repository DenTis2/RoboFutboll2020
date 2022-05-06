int D = 44;
int coff = 200;
int oldjoy = 1500;
int oldmotor_b = 1500;
int oldmotor_a = 1500;
int oldservo = 44;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 Serial.print("D");
 Serial.println(44);
 Serial.print("C");
 Serial.println(1500);
 Serial.print("B");
 Serial.println(1500);
 Serial.print("A");
 Serial.println(1500);
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
 int button1_camera_up = digitalRead(4);
 int button2_camera_centre = digitalRead(5);
 int button3_camera_dn = digitalRead(3);
 int button4_manipulator_shvat = digitalRead(2);
 int button5_manipulanor_rashvat = digitalRead(6);
 int button6_power_plus = digitalRead(9);
 int button7_power_standart = digitalRead(8);
 int button8_power_minus = digitalRead(7);
 if(button4_manipulator_shvat == 1){
  Serial.print("E");
  Serial.print(85);
  Serial.println(' ');
  while(button4_manipulator_shvat == 1){
  delay(10);
  button4_manipulator_shvat = digitalRead(5);
  }
 }
 if(button5_manipulanor_rashvat == 1){
  Serial.print("E");
  Serial.print(-85);
  Serial.println(' ');
  while(button5_manipulanor_rashvat == 1){
  delay(10);
  button5_manipulanor_rashvat = digitalRead(6);
  }
 }
 if(button2_camera_centre == 1){
  D = 44;
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
 if(abs(oldservo - D) > 1){
  Serial.print("D");
  Serial.print(D);
  Serial.println(' ');
  oldservo = D;
 }
 if (button7_power_standart == 1){
  coff = 200;
 }
 if(button6_power_plus == 1){
  coff = coff + 50;
  while(button6_power_plus == 1){
  delay(10);
  button6_power_plus = digitalRead(9);
  }
  //Serial.print("Button plus ");
  //Serial.println(coff);
 }
 /*if(button8_power_minus == 1){
  coff = coff - 50;
  while(button8_power_minus == 1){
  delay(10);
  button8_power_minus = digitalRead(7);
  }
 // Serial.print("Button minus ");
 // Serial.println(coff);
 }*/
 if (coff > 300){
  coff = 300;
 }
 if(coff < 0){
  coff = 0;
 }
 int joy_UpDn = analogRead(A0); 
 int joy_LeftRight = analogRead(A2);
 int joy_ForwardBack = analogRead(A1);
 int joy1 = map(joy_UpDn,920,135,1100 + coff,1900 - coff);
 int joy2 = map(joy_ForwardBack,0,910,1100 + coff,1900 - coff);
 int joy3 = map(joy_LeftRight,0,910,1100 + coff,1900 - coff);
 /*Serial.print(joy_UpDn);
 Serial.print(" ");
 Serial.print(joy_ForwardBack);
 Serial.print(" ");
 Serial.println(joy_LeftRight);*/
 joy1 = (joy1 + oldjoy) / 2;
 if(joy1<1525 && joy1>1475){
 joy1 = 1500;
 }
 if(abs(oldjoy - joy1) > 5){
  Serial.print("C");
  Serial.print(joy1);
  Serial.println(" ");
  oldjoy = joy1;
 }
 int motor_a = joy3 - joy2 + 1500;
 int motor_b = joy2 + joy3 - 1500;
 if (motor_b > 1900 - coff){
  motor_b = 1900 - coff;
 }
 if(motor_a > 1900 - coff){
  motor_a = 1900 - coff;
 }
 if (motor_b < 1100 + coff){
  motor_b = 1100 + coff;
 }
 if(motor_a < 1100 + coff){
  motor_a = 1100 + coff;
 }
 motor_a = (motor_a + oldmotor_a) / 2;
 if(motor_a<1525 && motor_a>1475){
 motor_a = 1500;
 }
 if(abs(oldmotor_a - motor_a) > 5){
 Serial.print("A");
 Serial.print(motor_a);
 Serial.println(" ");
 oldmotor_a = motor_a;  
 }
 motor_b = (motor_b + oldmotor_b) / 2;
 if(motor_b<1525 && motor_b>1475){
 motor_b = 1500;
 }
 if (abs(oldmotor_b - motor_b) > 5){
  Serial.print("B");
  Serial.print(motor_b);
  Serial.println(" ");
   Serial.print("B");
  Serial.print(motor_b);
  Serial.println(" ");
  oldmotor_b = motor_b;
 }
 delay(100);
}
