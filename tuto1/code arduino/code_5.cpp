int cmdVD;
int cmdVG;
int cmdD;
int cmdG;
int acc = 50;
char incomingByte ;

unsigned long previousMillis;
unsigned long previousMillis2;
unsigned long chrono;


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);  //Init the Serial baudrate

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
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = (char)Serial.read();
    if( incomingByte == 'A'){
      cmdVD = 255;
      cmdVG = 255;
    }
    if( incomingByte == 'R'){
      cmdVD = -255;
      cmdVG = -255;
    }
    if( incomingByte == 'D'){
      cmdVD = -255;
      cmdVG = 255;
    }
    if( incomingByte == 'G'){
      cmdVD = 255;
      cmdVG = -255;
    }
    if( incomingByte == 'S'){
      cmdVD = 0;
      cmdVG = 0;
    }
    Serial.println(incomingByte);
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
