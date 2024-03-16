
#include "SabertoothSimplified.h"
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(6, 7); // RX on pin 6 (unused), TX on pin 7 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

//SabertoothSimplified ST; // We'll name the Sabertooth object ST.
// For how to configure the Sabertooth, see the DIP Switch Wizard for
//   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
// Be sure to select Simplified Serial Mode for use with this library.
// This sample uses a baud rate of 9600.
//
// Connections to make:
//   Arduino TX->1  ->  Sabertooth S1
//   Arduino GND  ->  Sabertooth 0V
//   Arduino VIN  ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//
// If you want to use a pin other than TX->1, see the SoftwareSerial example.                                                                 


// pin
#define interruptPinRA 2
#define interruptPinLA 3
#define interruptPinRB 4
#define interruptPinLB 5

// tick / cm
#define tickcmR 58.8
#define tickcmL 59.1

//tick / rad horaire
#define tickZR_P 882
#define tickZL_N 886

//tick / rad trigo
#define tickZL_P 887
#define tickZR_N 882


// parametre K
#define KRap 1.3
#define KRai 0.4

#define Kap 0.8
#define Kai 0.25

#define Kdp 1
#define Kdi 0.2

// correction gauche
#define KL 10

// vitesse max
#define maxSpeed 120
#define minSpeed 10
#define maxAcc 15

// pour le controle manuel
#define coeff 600
#define P 0.5
#define I 0.1
#define D (-0.02)

// String streamChar;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data
// position du robot
float X = 0;
float Y = 0;
float Z = 0;

// position de la cible
float Xt = 0;
float Yt = 0;
float Zt = 0;

// chronometrage
unsigned long interval = (unsigned long) 50;
unsigned long previousMillis;

// compte codeur 
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

// rampte de vitesse
const float rankSpeed = 10;
float cmdSpeed;

// precision de position a atteindre
const float precisionPos = 0.5; //cm
const float precisionZ = PI/360; //0.5 deg

float IEz, Idc;
bool goBack = true;
int rotSigne = 0;
int loopCount;
char cmd;
int cmdR,cmdL;
int prevCmdR,prevCmdL,prevCmd;

// pour le controle manuel
int cmdVR,cmdVL;
float PErrR, PErrL;
long IErrR, IErrL;


void setup() {
  
  Serial.begin(115200);  //Init the Serial baudrate
  SWSerial.begin(9600);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  
  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);
  
  ST.stop();
  delay(1000);     // Wait 2 seconds.
  
  long countR = 0;
  long countL = 0;
  float X = 0;
  float Y = 0;
  float Z = 0;
  
  cmdSpeed = 70;

  previousMillis = millis();
  
}

void loop() {
    // delta odometrie
    float dR, dL, dD, dZ;
    float ErrR, ErrL;
    // calcul cible
    float dc,Ddc, signe, zc ;
    float Ez, DEz;
               
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
    loopCount=10;
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

    // on projete sur X et Y
    X += dD * cos(Z + dZ/2);
    Y += dD * sin(Z + dZ/2);

    // on ajout a Z
    Z  += dZ;
    Ez = 0;
    if(Z > PI ){
        Z=Z-2*PI;
    }
    if(Z < -PI ){
        Z=Z+2*PI;
    }
    
    Serial.print(X);
    Serial.print(":");
    Serial.print(Y);
    Serial.print(":");
    Serial.print(Z*180/PI);
    
    // si non action de mouvement
    if (cmd==' '){ 
        Serial.print(":1");
        digitalWrite(13, 1);
        // on envoie un stop
        ST.stop();
 
    // recallage
    }else if (cmd=='C' or cmd=='F'){
        
        if (cmd=='C'){
            if (goBack) {
                cmdR = -20;
                cmdL = -20;
            }else{
                cmdR = 20;
                cmdL = 20;
            }
        }else{

            if (goBack) {
                cmdR = -30;
                cmdL = -30;
            }else{
                cmdR = 30;
                cmdL = 30;
            }
        }
        
        // on calcul d'angle entre le robot et la cible
        Ez = Zt - Z;
        
        if(cmd=='C' and (dR==0 or dL==0)) Ez = 0; 
        // si on change de sens, on raz l'integrale pour éviter de surtourner
        if (Ez>0){
            if (rotSigne != 1){
                rotSigne = 1;
                IEz=0;
            }
        }else{
            if (rotSigne != -1){
                rotSigne = -1;
                IEz=0;
            }
        }

        // calcul de correction d'angle a faire
        IEz += Ez;    
        DEz = Kap * Ez + Kai * IEz;
        cmdR += (int)(DEz*cmdSpeed);
        cmdL -= (int)(DEz*cmdSpeed);

        // si les roue ne bouge plus, on est contre le bors de la table
        if(dR==0 and dL==0){
            if (loopCount==0){
                // on envoie un stop
                ST.stop();
                Serial.print(":1");
                digitalWrite(13, 1);
                // on évite de nouveau déplacement
                cmd=' ';
                countR = 0;
                countL = 0;
                IEz=0;
                prevCountR = 0;
                prevCountL = 0;
                ST.motor(1, 0);
                ST.motor(2, 0);
            }else{
                Serial.print(":0");
                loopCount--;
                ST.motor(1, cmdR);
                ST.motor(2, cmdL);
            }
        }else{
            //digitalWrite(ledPin, LOW);
            // on envoie les commande à la SB
            Serial.print(":0");
            ST.motor(1, cmdR);
            ST.motor(2, cmdL);
            loopCount = 10;
        }
    // controle Manuel
    }else if(cmd=='M'){
        // on divise par deux, car les commandes SB sont sur -127 à 127, alors que la commande manuelle est sur -255 à 255
        cmdR = cmdVR/2;
        cmdL = cmdVL/2;
        // si stop
        if (cmdR == 0){
            PErrR =0;
            IErrR =0;
        // si changement de sens
        }else if( prevCmdR/cmdR <0 ){
            PErrR =0;
            IErrR =0;
        }
    
        // si stop
        if (cmdL == 0){
            PErrL =0;
            IErrL =0;
        // si changement de sens
        }else if( prevCmdR/cmdL <0 ){
            PErrL =0;
            IErrL =0;
        }

        // on limite l'acceleration
        if ((abs(cmdR-prevCmdR))>maxAcc){
          cmdR = prevCmdR+ maxAcc * (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
        }
        if ((abs(cmdL-prevCmdL))>maxAcc){
          cmdL = prevCmdL+ maxAcc * (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
        }
        prevCmdR = cmdR;
        prevCmdL = cmdL;
        
        dR = dR*coeff;
        dL = dL*coeff;
        /*
        ErrR = cmdR-dR;
        IErrR += ErrR;
        cmdR = P * ErrR + I * IErrR + D *(ErrR-PErrR) ;
        PErrR = ErrR;
        
        ErrL = cmdL-dL;
        IErrL += ErrL;
        cmdL = P * ErrL + I * IErrL + D *(ErrL-PErrL) ;
        PErrL = ErrL; /**/
        if( cmdR > 125 ) cmdR = 125;
        if( cmdR < -125 ) cmdR = -125;
        if( cmdL > 125 ) cmdL = 125;
        if( cmdL < -125 ) cmdL = -125;

        ST.motor(1, cmdR);
        ST.motor(2, cmdL);
        
    }else {
        //mouvement
        if (cmd=='G' or cmd=='T'){
            
            // distance à la cible
            dc = sqrt((Xt-X)*(Xt-X)+(Yt-Y)*(Yt-Y));
            Idc += dc;
            if (abs(dc) > precisionPos*10 ){
                Ddc = Kdp * dc;
                Idc = 0;
            }else{
                Ddc = Kdp * dc + Kdi * Idc;
            }
    
            //On regarde si la cible est à gauche ou à droite du robot
            if(Y > Yt){
                signe = -1;
            }else{
                signe = 1;
            }

            //On calcule l'angle a la cible
            zc = signe * acos((Xt-X)/dc);
            // on calcul d'angle entre le robot et la cible
            Ez = zc - Z;

            if(Ez > PI ){
                Ez=Ez-2*PI;
            }
            if(Ez < -PI ){
                Ez=Ez+2*PI;
            }

            // si la différence est >90, on part en marche arriere
            if(abs(Ez)>(PI/2)){
                Ddc  = -Ddc;
                // on retourne l'angle de cible pour rester en marche arriere
                if(Ez<0){
                    Ez = Ez+PI;
                }else{
                    Ez = Ez-PI;
                }
                digitalWrite(12, 1);
            }else{
                digitalWrite(12, 0);
            }

            
            // si on a un trop grand angle a faire, non transit
            if(abs(Ez) > PI/8 and cmd!='T'){
              // Serial.print(": RR ");
              DEz = Kap * Ez;
              // on ne fait que tourner
              cmdR = (int)(DEz*cmdSpeed);
              cmdL = (int)(-DEz*cmdSpeed);
            }else{

              // Serial.print(": GG ");
              // Serial.print(dc);
              // Serial.print(": Ddc ");
              // Serial.print(Ddc);
              
              cmdR  = (int)(Ddc*rankSpeed);
              // Serial.print(": cmdR1 ");
              // Serial.print(cmdR);

              // on plafone a la vitesse de commande
              if(cmdR>cmdSpeed){
                  cmdR = cmdSpeed;
              }
              if(cmdR<-(cmdSpeed)){
                  cmdR = -(cmdSpeed);
              }

              // on limite l'acceleration (mais pas la deceleration)
              prevCmd = (prevCmdR+ prevCmdL)/ 2;
              // Serial.print(": prevCmd ");
              // Serial.print(prevCmd);

              if ((abs(cmdR)-abs(prevCmd))>maxAcc){
                  cmdR = prevCmd+ maxAcc *(cmdR-prevCmd)/ abs(cmdR-prevCmd);
              }

              cmdL = cmdR;

              // calcul de correction d'angle a faire
              IEz += Ez;
              if (abs(Ez) > PI/20 ){
                  DEz = Kap * Ez;
                  IEz = 0;
              }else{
                  DEz = Kap * Ez + Kai * IEz;
              }
              // Serial.print(": cmdR3 ");
              // Serial.print(cmdR);
              // Serial.print(": cmdL3 ");
              // Serial.print(cmdL);
              // Serial.print(": DEz ");
              // Serial.print(DEz);
              cmdR += (int)(DEz*abs(cmdR));
              cmdL -= (int)(DEz*abs(cmdL));  
              // Serial.print(": cmdR4 ");
              // Serial.print(cmdR);
              // Serial.print(": cmdL4 ");
              // Serial.print(cmdL);

              if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
                  cmdR = prevCmdR+ maxAcc *(cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
              }
              if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
                  cmdL = prevCmdL+ maxAcc *(cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
              }
              // Serial.print(": cmdR5 ");
              // Serial.print(cmdR);
              // Serial.print(": cmdL5 ");
              // Serial.print(cmdL);
          }
        //rotation
        }else if(cmd=='R'){
            dc=0;

            // si on est pret de l'arrive, on se tourne dans la bonne direction
            cmdR = 0;
            cmdL = 0;
            Ez = Zt - Z;

            if(Ez > PI ){
                Ez=Ez-2*PI;
            }
            if(Ez < -PI ){
                Ez=Ez+2*PI;
            }                    
            // si on change de sens, on raz l'integrale pour éviter de surtourner
            if (Ez>0){
                if (rotSigne != 1){
                    rotSigne = 1;
                    IEz=0;
                }
            }else{
                if (rotSigne != -1){
                    rotSigne = -1;
                    IEz=0;
                }
            }
            // calcul de correction d'angle a faire
            IEz += Ez;
            if (abs(Ez) > PI/20 ){
                DEz = KRap * Ez;
                IEz = 0;
            }else{
                DEz = KRap * Ez + KRai * IEz;
            }
            
            cmdR = (int)(DEz*cmdSpeed);
            
            if(cmdR>cmdSpeed){
                cmdR = cmdSpeed;
            }
            if(cmdR<-(cmdSpeed)){
                cmdR = -(cmdSpeed);
            }
            
            cmdL = -cmdR;
            // DEz - retard gauche
            cmdR = cmdR + KL * (-dR - dL)*cmdSpeed;
            // DEz + retard gauche
            cmdL = cmdL + KL * (-dR - dL)*cmdSpeed;
            
            if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
                cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
            }
            if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
                cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
            }
            
        }else{
            IEz=0;
            Idc = 0;
        }
        
        prevCmdR = cmdR;
        prevCmdL = cmdL;

        /* moteur R : 1 > 64 > 127
        moteur L : 128 > 192 > 255 */
        if(cmdR>maxSpeed){
            cmdR = maxSpeed;
        } else if(cmdR<-maxSpeed){
            cmdR = -maxSpeed;
        }
        if(cmdL>maxSpeed){
            cmdL = maxSpeed;
        }else if(cmdL<-maxSpeed){
            cmdL = -maxSpeed;
        }

        // vitesse minimal pour la phase d'approche
        if (abs(cmdR) < minSpeed) cmdR = minSpeed* (cmdR)/ abs(cmdR);
        if (abs(cmdL) < minSpeed) cmdL = minSpeed* (cmdL)/ abs(cmdL);

        Serial.print(":");
        
        // si on est arrivé depuis assez longtemps
        if (loopCount<=0){
            cmd=' ';
            // on envoie un stop
            ST.stop();
            // on raz l'integral
            IEz = 0;
            Idc = 0;
            prevCmdR = 0;
            prevCmdL = 0;
            Serial.print("1");
            digitalWrite(13, 1);
        // si on est très proche du but, on s'arrete et on reduit le compte pls vite
        }else if((abs(dc) < precisionPos and (cmd=='G' or cmd=='T')) or ( cmd=='R' and  abs(Ez) < precisionZ )){
            loopCount-=2;
            // on envoie un stop
            ST.stop();
            // on raz l'integral
            IEz = 0;
            Idc = 0;
            prevCmdR = 0;
            prevCmdL = 0;
            Serial.print("0");
            digitalWrite(13, 0);
        
        // si on est proche du but, on continue et on réuit le compte
        }else if((abs(dc) < precisionPos*4 and (cmd=='G' or cmd=='T')) or ( cmd=='R' and  abs(Ez) < precisionZ*4 )){
            loopCount-=1;

            // on envoie les commande à la SB
            ST.motor(1, cmdR);
            ST.motor(2, cmdL);
            Serial.print("0");
            digitalWrite(13, 0);
        
        // si on n'est pas encore au but, on continue et on met le compte à 10
        }else{
            loopCount=10;
            // on envoie les commande à la SB
            ST.motor(1, cmdR);
            ST.motor(2, cmdL);

            // transite on envoie le retour à 10 cm de la cible, avant de ralentir
            if(abs(dc) < 10 and cmd=='T'){
                Serial.print("1");
                digitalWrite(13, 1);
            }else{
                Serial.print("0");
                digitalWrite(13, 0);
            }
        }
    }
    Serial.println("");
  }
}



void decryptIncom(){
    bool neg;
    i = 0;
    loopCount = 10;
    digitalWrite(13, 0);
    
    // recallage
    // C:+/-
    // F:+/-
    if (streamChar[0] == 'C' or streamChar[0]=='F' ) {
        goBack = !(streamChar[2] == '+');
        cmd = streamChar[0];
        Zt = Z;
    }
    
    // goto destination ou transite destination
    // G:xxx:yyy
    // T:xxx:yyy
    if (streamChar[0] == 'G' or streamChar[0]=='T') {
        cmd = streamChar[0];
        i = 2;
        Xt = 0;
        
        while (isDigit(streamChar[i])) {
            Xt = Xt * 10 + streamChar[i] - '0';
            i++;
        }
    
        i++;
        Yt = 0;
        while (isDigit(streamChar[i])) {
            Yt = Yt * 10 + streamChar[i] - '0';
            i++;
        }
    }
    
    // rotation à l'angle
    // R:zzz
    if (streamChar[0] == 'R') {
        cmd = streamChar[0];
        i = 2;
        Zt = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Zt = Zt * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Zt = -Zt;
        Zt = Zt*PI/180;
    }
    
    // stop
    // 0
    if (streamChar[0] == '0') {
        cmdSpeed = 0;
    }
    
    // commande de vitesse
    // V:vvv
    if (streamChar[0] == 'V') {
        cmdSpeed = 0;
        i = 2;
        while (isDigit(streamChar[i])) {
            cmdSpeed = cmdSpeed * 10 + streamChar[i] - '0';
            i++;
        }
        if(cmdSpeed>maxSpeed) cmdSpeed = maxSpeed;
    }
    
    // set de la position
    // S:xxx:yyy:zzz
    if (streamChar[0] == 'S') {
        countR = 0;
        countL = 0;
        prevCountR = 0;
        prevCountL = 0;
        i = 2;
        X = 0;
        while (isDigit(streamChar[i])) {
            X = X * 10 + streamChar[i] - '0';
            i++;
        }
        Xt = X;
        i++;
        Y = 0;
        while (isDigit(streamChar[i])) {
            Y = Y * 10 + streamChar[i] - '0';
            i++;
        }
        Yt = Y;
    
        i++;
          
        Z = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Z = Z * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Z = -Z;
        Z = Z*PI/180;
        Zt = Z;
    }
    
    // set de la position sur X
    // X:xxx
    if (streamChar[0] == 'X') {
        i = 2;
        X = 0;
        while (isDigit(streamChar[i])) {
            X = X * 10 + streamChar[i] - '0';
            i++;
        }
        Xt = X;
    }
    
    // set de la position sur Y
    // Y:yyy
    if (streamChar[0] == 'Y') {
        i = 2;
        Y = 0;
        while (isDigit(streamChar[i])) {
            Y = Y * 10 + streamChar[i] - '0';
            i++;
        }
        Yt = Y;
    }
    
    // set de la position sur Z
    // Z:zzz
    if (streamChar[0] == 'Z') {
        i = 2;
        Z = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Z = Z * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Z = -Z;
        Z = Z*PI/180;
        Zt = Z;
    }
    // Mode Manuel
    // M:ggg:ddd
    if (streamChar[0] == 'M') {
      cmd = streamChar[0];
      i = 2;
      cmdVR = 0;
      cmdVL = 0;
      if (streamChar[i] == '-'){
        neg = true;
        i++;
      }else neg = false;
      while (isDigit(streamChar[i])) {
        cmdVL = cmdVL * 10 + streamChar[i] - '0';
        i++;
      }
      if (neg) cmdVL = -cmdVL;
      i++;

      if (streamChar[i] == '-'){
        neg = true;
        i++;
      }else neg = false;
      while (isDigit(streamChar[i])) {
        cmdVR = cmdVR * 10 + streamChar[i] - '0';
        i++;
      }
      if (neg) cmdVR = -cmdVR;
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


