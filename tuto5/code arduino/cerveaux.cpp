
#include <SoftwareSerial.h>
SoftwareSerial motor_Serial(6, 7); // RX on pin 6, TX on pin 7.
bool matchPrepare = true;
bool matchRunning = false;
bool matchEnd = false;
unsigned long startMillis;
unsigned long matchTime = (unsigned long) 10000; // 10 000 ms
char vitesse[] = "V:70";

void setup() {
  Serial.begin(115200);  //Init the Serial baudrate
  motor_Serial.begin(115200);
  pinMode(13,INPUT_PULLUP );
  pinMode(8,INPUT );
  pinMode(2,INPUT );

  attachInterrupt(digitalPinToInterrupt(2), evitement, CHANGE);
  Serial.println("Ready");

  wait(0);
  Serial.println("Initialisation");
  motor_cmd(vitesse);
  motor_cmd("C:-");
  motor_cmd("S:12:40:0");
  motor_cmd("G:25:40");
  matchPrepare = false;

  wait(1);
  Serial.println("Match");
  matchRunning = true;
  startMillis = millis();
  motor_cmd("G:60:40");
  motor_cmd("R:90");
  motor_cmd("G:60:80");
  motor_cmd("G:40:80");
  motor_cmd("G:40:40");
  
  matchRunning = false;
  Serial.println("Fin");
  matchEnd = true;
  motor_cmd("G:30:30");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
 // Serial.println(digitalRead(13));
}

void motor_cmd(char command[]) {
  Serial.println(command);

  if(matchRunning && millis()-startMillis >= matchTime){
    matchRunning = false;
  }
  if(matchRunning || matchPrepare || matchEnd){
    char streamChar[32] ;
    int i;
    int incomingByte = 0;

    motor_Serial.println(command);
    delay(500);
    int status;
    status = 0;

    while(status != 1){
      delay(100);
      status = digitalRead(8);
     // Serial.println(status);
    }
  }
}


void wait(int status) {
  while(digitalRead(13) != status){
    delay(100);
  }
  
}


void evitement()
{
  if (digitalRead(2) == HIGH) {  
    Serial.println("0");
    motor_Serial.println("0");
  }else{
    Serial.println(vitesse);
    motor_Serial.println(vitesse);
  }
}
