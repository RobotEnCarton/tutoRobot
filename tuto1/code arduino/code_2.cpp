int cmdV;
unsigned long previousMillis;
unsigned long chrono;


void setup() {
  // put your setup code here, to run once:
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  previousMillis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  chrono = millis() - previousMillis;
  
  if( chrono>0 & chrono <1000){
    cmdV = 255;
  }

  if( chrono>1000 & chrono <2000){
    cmdV = 0;
  }

  if( chrono>2000 & chrono <3000){
    cmdV = -255;
  }

  if( chrono>3000 & chrono <8000){
    cmdV = 0;
  }

  if(chrono >8000){
    previousMillis = millis();
  }
  sendCmd(cmdV);
}

void sendCmd(int cmdV){

  if (cmdV>0){
    digitalWrite(3,HIGH);
    digitalWrite(6,HIGH);
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
    analogWrite(9,cmdV);
    analogWrite(10,cmdV);

  }
  if(cmdV<0){
    digitalWrite(3,LOW);
    digitalWrite(6,LOW);
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    analogWrite(9,abs(cmdV));
    analogWrite(10,abs(cmdV));
    
  }

  if(cmdV==0){
    digitalWrite(3,LOW);
    digitalWrite(6,LOW);
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
    digitalWrite(9,0);
    digitalWrite(10,0);
    
  }

}
