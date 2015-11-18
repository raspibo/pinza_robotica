/********************************************************
 * Progetto Pinza Robotica - un esperimento del gruppo robottini di Raspibo 
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=.5;
double consKp=1, consKi=0.05, consKd=0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


int en12=9;
int en34=10;
int dir12=4;
int dir34=5;
int motor=0;
int board=1;
int out_bridge_n=12;
int incomingByte = 0;   // for incoming serial data
int s_speed=55;
int x=0;
int sensorPin = A0;
int sensorValue = 0;
int l_position = 0;
unsigned long time; 
int op;
int dir=0;
int Out;
double INMin=320;
double INMax=790;

void setup()
{
  Serial.begin(115200);
  //initialize the variables we're linked to
  Input = analogRead(0);
  Setpoint = Input;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  pinMode(en12, OUTPUT);
  pinMode(dir12, OUTPUT);
  pinMode(en34, OUTPUT);
  pinMode(dir34, OUTPUT); 
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(50);
}

void loop()
{
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    switch(incomingByte) {
    case '+': 
      if (Setpoint<790) {
        Setpoint = Setpoint+15;
      }
      break;
    case '-': 
      if (Setpoint>320) {
        Setpoint = Setpoint-15;
      }  
      break;
    }
  }  
  Input = analogRead(A0);
  double gap = abs(Setpoint-Input); //distance away from setpoint
  //if(gap<10) {  //we're close to setpoint, use conservative tuning parameters
  myPID.SetTunings(consKp, consKi, consKd);
  //} 
  //else { //we're far from setpoint, use aggressive tuning parameters
  //  myPID.SetTunings(aggKp, aggKi, aggKd);
  //}
  if (gap>5) {
    myPID.Compute();
    if (Output>0) {
      dir=0;
    } 
    else {  
      dir=1;
    }  
    digitalWrite(4, dir);
    Out=abs(Output);
    Out=map(abs(Output), 0, 255, 30, 255);
    analogWrite(9,Out);
  } else {
    analogWrite(9,0);
  }  
  x++;
  if (x>=1000) { 
    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(" - Input: ");
    Serial.print(Input);
    Serial.print(" - Out: ");
    Serial.print(Out);
    Serial.print(" - dir: ");
    Serial.print(dir);
    Serial.print(" - gap: ");
    Serial.println(gap);
    x=0;
  }
}





