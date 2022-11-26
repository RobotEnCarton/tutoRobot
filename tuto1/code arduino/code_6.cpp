int cmdVD;
int cmdVG;
int cmdD;
int cmdG;
int acc = 50;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data

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
  
  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    streamChar[i] = (char)incomingByte;
    i++;
  }

  if (incomingByte == 10){
    incomingByte =0;
    decryptIncom();
    i = 0;
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
    sendCmd(cmdD,cmdG );
    Serial.print(cmdVD);
    Serial.print(":");
    Serial.println(cmdVG);
  }


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

void decryptIncom(){
  bool neg;
  i = 0;
  cmdVD = 0;
  cmdVG = 0;
  if (streamChar[i] == '-'){
    neg = true;
    i++;
  }else neg = false;
  while (isDigit(streamChar[i])) {
    cmdVD = cmdVD * 10 + streamChar[i] - '0';
    i++;
  }
  if (neg) cmdVD = -cmdVD;
  i++;

  if (streamChar[i] == '-'){
    neg = true;
    i++;
  }else neg = false;
  while (isDigit(streamChar[i])) {
    cmdVG = cmdVG * 10 + streamChar[i] - '0';
    i++;
  }
  if (neg) cmdVG = -cmdVG;
}
