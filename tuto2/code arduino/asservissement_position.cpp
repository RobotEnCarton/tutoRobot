
#include "SabertoothSimplified.h"
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(6, 7); // RX on pin 6 (unused), TX on pin 7 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.



// pin
#define interruptPinRA 2
#define interruptPinLA 3
#define interruptPinRB 4
#define interruptPinLB 5

// tick / cm
#define tickcmR 58.6
#define tickcmL 58.9

//tick / rad horaire
#define tickZR_P 882
#define tickZL_N 886

//tick / rad trigo
#define tickZL_P 887
#define tickZR_N 882


// parametre K
#define KRap 1.3
#define KRai 0.4

#define Kap 1
#define Kai 0

#define Kdp 1
#define Kdi 0

// vitesse max
#define maxSpeed 120
#define minSpeed 10
#define maxAcc 10

// compte codeur 
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

float dT = 0;
float IdT = 0;
float zT = 0;
float IzT = 0;
float cmdV = 0;
char cmd;
int cmdR;
int cmdL;
int prevCmdR;
int prevCmdL;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data

unsigned long previousMillis;
unsigned long chrono;


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);  //Init the Serial baudrate
  SWSerial.begin(9600);
  
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
    dR = (float)(countR - prevCountR); // (float)(dc);
    prevCountR = countR;
    dL = (float)(countL - prevCountL); // (float)(dc);
    prevCountL = countL;
    
    // on fait le delta de distance et d'angle
    dD = (dR/tickcmR+dL/tickcmL)/2;
    
    
    if(dR>0){
        dR = dR/tickZR_P;
    }else{
        dR = dR/tickZR_N;
    }
    
    if(dL>0){
        dL = dL/tickZL_P;
    }else{
        dL = dL/tickZL_N;
    }
    
    dZ = (dR -dL)/2;

    dT -= dD;
    zT -= dZ;

    Serial.print("dT ");
    Serial.print(dT);
    Serial.print("  :zT ");
    Serial.print(zT*180/PI);
    
    // si non action de mouvement
    if (cmd==' '){ 
        Serial.print(":1");
        // on envoie un stop
        ST.stop();
    }
    // recallage
    else if (cmd=='d'){
      IdT += dT;
      cmdV = Kdp * dT + Kdi * IdT;
      cmdR  = (int)(cmdV*maxAcc);

      // on plafone a la vitesse de commande
      if(cmdR>maxSpeed){
          cmdR = maxSpeed;
      }
      if(cmdR<-(maxSpeed)){
          cmdR = -(maxSpeed);
      }
      cmdL = cmdR;

      // on limite l'acceleration
      if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
          cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
          cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }
      prevCmdR = cmdR;
      prevCmdL = cmdL;
      
      IzT += zT;
      if (abs(zT) > PI/20 ){
          Ez = KRap * zT;
          IzT = 0;
      }else{
        Ez = KRap * zT + KRai * IzT;
      }
      cmdR += (int)(Ez*abs(cmdR));
      cmdL -= (int)(Ez*abs(cmdL));   

      // on envoie les commande à la SB
      ST.motor(1, cmdR);
      ST.motor(2, cmdL);
    }

    else if (cmd=='r'){
      
      IzT += zT;
      if (abs(zT) > PI/20 ){
          cmdV = KRap * zT;
          IzT = 0;
      }else{
        cmdV = KRap * zT + KRai * IzT;
      }
      cmdR  = (int)(cmdV*maxSpeed);
      // on plafone a la vitesse de commande
      if(cmdR>maxSpeed){
          cmdR = maxSpeed;
      }
      if(cmdR<-(maxSpeed)){
          cmdR = -(maxSpeed);
      }
      cmdL = -cmdR;

      // on limite l'acceleration
      if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
          cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
          cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }
      prevCmdR = cmdR;
      prevCmdL = cmdL;
    
      // on envoie les commande à la SB
      ST.motor(1, cmdR);
      ST.motor(2, cmdL);
    }

    Serial.println("");
  }
}

void decryptIncom(){
  bool neg;
  i = 0;
  cmd = streamChar[i];
  if (streamChar[i] == '0'){
    countR=0;
    countL=0;
    dT = 0;
    zT = 0;
  }

  if (streamChar[i] == 'd'){
    i = 2;
    dT = 0;
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;
    while (isDigit(streamChar[i])) {
      dT = dT * 10 + streamChar[i] - '0';
      i++;
    }
  }
  if (streamChar[i] == 'r'){
    i = 2;
    zT = 0;
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;
    while (isDigit(streamChar[i])) {
      zT = zT * 10 + streamChar[i] - '0';
      i++;
    }
    if (neg) zT = -zT;
    zT = zT*PI/180;
  }
}


void interruptR()
{
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) {  
    countR--;
  }else{
    countR++;
  }
}

void interruptL()
{
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) {  
    countL++;
  }else{
    countL--;
  }
}


