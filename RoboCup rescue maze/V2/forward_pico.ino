int valr=128;
int valrb=128;
int vall=128;
int vallb=128;
int naprr=1;
int naprrb=1;
int naprl=1;
int naprlb=1;
void setup() {
  // put your setup code here, to run once:
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
  analogWrite(5,valr);
  digitalWrite(8,1-naprr);
  digitalWrite(2,naprr);
  analogWrite(3,valrb);
  digitalWrite(7,naprrb);
  digitalWrite(1,naprrb);
  analogWrite(9,vallb);
  digitalWrite(0,1-naprlb);
  digitalWrite(A7,1-naprlb);
  analogWrite(11,vall);
  digitalWrite(A1,naprl);
  digitalWrite(A2,1-naprl);

  
}
