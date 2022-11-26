int cmdVD;
int cmdVG;
int cmdD;
int cmdG;
int acc = 50;
unsigned long previousMillis;
unsigned long previousMillis2;
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
  previousMillis2 = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  chrono = millis() - previousMillis;
  
  if( chrono>0 & chrono <1000){
    cmdVD = 255;
    cmdVG = 255;
  }

  if( chrono>1000 & chrono <2000){
    cmdVD = 0;
    cmdVG = 0;
  }

  if( chrono>2000 & chrono <3000){
    cmdVD = -255;
    cmdVG = -255;
  }

  if( chrono>3000 & chrono <4000){
    cmdVD = 0;
    cmdVG = 0;
  }
  
  if( chrono>4000 & chrono <5000){
    cmdVD = -128;
    cmdVG = 128;
  }
  
  if( chrono>5000 & chrono <6000){
    cmdVD = 0;
    cmdVG = 0;
  }

  if( chrono>6000 & chrono <7000){
    cmdVD = 128;
    cmdVG = -128;
  }

  if( chrono>7000){
    cmdVD = 0;
    cmdVG = 0;
  }

  if(chrono >8000){
    previousMillis = millis();
  }

  if( millis() - previousMillis2 >100){
    previousMillis2 = millis();
    if( cmdD > cmdVD ){
      cmdD -= acc;
      cmdD = max(cmdVD, cmdD) ;
    }
    if( cmdD < cmdVD ){
      cmdD += acc;
      cmdD = min(cmdVD, cmdD) ;
    }    

    if( cmdG > cmdVG ){
      cmdG -= acc;
      cmdG = max(cmdVG, cmdG) ;
    }
    if( cmdG < cmdVG ){
      cmdG += acc;
      cmdG = min(cmdVG, cmdG) ;
    }
  }


  sendCmd(cmdD,cmdG );
}

void sendCmd(int cmdD,int cmdG){

  if (cmdD>0){
    digitalWrite(3,HIGH);
    digitalWrite(4,LOW);
    analogWrite(9,cmdD);
  }
  if (cmdG>0){
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    analogWrite(10,cmdG);
  }

  if(cmdD<0){
    digitalWrite(3,LOW);
    digitalWrite(4,HIGH);
    analogWrite(9,abs(cmdD));
  }
  if (cmdG<0){
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    analogWrite(10,abs(cmdG));
  }

  if(cmdD==0){
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    analogWrite(9,0);
  }

  if(cmdG==0){
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    analogWrite(10,0);    
  }

}