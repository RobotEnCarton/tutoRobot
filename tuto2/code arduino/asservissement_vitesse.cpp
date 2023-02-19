
// pin
#define interruptPinRA 2
#define interruptPinLA 3
#define interruptPinRB 4
#define interruptPinLB 5

#define pinMotorRA 6
#define pinMotorRB 7
#define pinMotorRS 10
#define pinMotorLA 8
#define pinMotorLB 9
#define pinMotorLS 11


// compte codeur 
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

float coeff = 1;
float P = 0.5;
float I = 0.3;
float D = -0.2;

float PErrR, PErrL;
long IErrR, IErrL;

int cmdVD;
int cmdVG;
int cmdD;
int cmdG;
int acc = 100;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data

unsigned long previousMillis;
unsigned long chrono;


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);  //Init the Serial baudrate

  pinMode(pinMotorRA ,OUTPUT);
  pinMode(pinMotorRB ,OUTPUT);
  pinMode(pinMotorRS ,OUTPUT);
  pinMode(pinMotorLA ,OUTPUT);
  pinMode(pinMotorLB ,OUTPUT);
  pinMode(pinMotorLS ,OUTPUT);
  
  
  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);
  
  
  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);
  
  previousMillis = millis();
}

void loop() {
    // delta odometrie
    float dR, dL, dD, dZ;
	float ErrR, ErrL;
  
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


  if( millis() - previousMillis >100){
    previousMillis = millis();
	
	// on fait le delta de chaque roue
	dR = (float)(countR - prevCountR)*coeff; // (float)(dc);
	prevCountR = countR;
	dL = (float)(countL - prevCountL)*coeff; // (float)(dc);
	prevCountL = countL;
		
    // Serial.print(dL);
    // Serial.print(":");
    // Serial.print(dR);
    // Serial.print(ErrR);
    // Serial.print("  :");
	Serial.println(dR);
	
	if (cmdVD == 0){
		PErrR =0;
		IErrR =0;
	}
	if (cmdVG == 0){
		PErrL =0;
		IErrL =0;
	}
	
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
	
	ErrR = cmdD-dR;
	IErrR += ErrR;
    // Serial.print(ErrR);
    // Serial.print(" =:");
    // Serial.print(IErrR);
    // Serial.print("  :");
	cmdD = P * ErrR + I * IErrR + D *(ErrR-PErrR) ;
	PErrR = ErrR;
	
	ErrL = cmdG-dL;
	IErrL += ErrL;
    // Serial.print(ErrR);
    // Serial.print(" =:");
    // Serial.print(IErrR);
    // Serial.print("  :");
	cmdG = P * ErrL + I * IErrL + D *(ErrL-PErrL) ;
	PErrL = ErrL;
	
    sendCmd(cmdG,cmdD );
	
    // Serial.print(cmdG);
    // Serial.print(":");
    // Serial.println(cmdD);
  }


}

void sendCmd(int cmdG,int cmdD){

  if (cmdD>0){
    digitalWrite(pinMotorRA,HIGH);
    digitalWrite(pinMotorRB,LOW);
    analogWrite(pinMotorRS,cmdD);
  }
  if (cmdG>0){
    digitalWrite(pinMotorLB,HIGH);
    digitalWrite(pinMotorLA,LOW);
    analogWrite(pinMotorLS,cmdG);
  }

  if(cmdD<0){
    digitalWrite(pinMotorRA,LOW);
    digitalWrite(pinMotorRB,HIGH);
    analogWrite(pinMotorRS,abs(cmdD));
  }
  if (cmdG<0){
    digitalWrite(pinMotorLB,LOW);
    digitalWrite(pinMotorLA,HIGH);
    analogWrite(pinMotorLS,abs(cmdG));
  }

  if(cmdD==0){
    digitalWrite(pinMotorRA,LOW);
    digitalWrite(pinMotorRB,LOW);
    analogWrite(pinMotorRS,0);
  }

  if(cmdG==0){
    digitalWrite(pinMotorLB,LOW);
    digitalWrite(pinMotorLA,LOW);
    analogWrite(pinMotorLS,0);    
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
    cmdVG = cmdVG * 10 + streamChar[i] - '0';
    i++;
  }
  if (neg) cmdVG = -cmdVG;
  i++;

  if (streamChar[i] == '-'){
    neg = true;
    i++;
  }else neg = false;
  while (isDigit(streamChar[i])) {
    cmdVD = cmdVD * 10 + streamChar[i] - '0';
    i++;
  }
  if (neg) cmdVD = -cmdVD;
}


void interruptR()
{
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) {  
    countR++;
  }else{
    countR--;
  }
}

void interruptL()
{
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) {  
    countL--;
  }else{
    countL++;
  }
}

