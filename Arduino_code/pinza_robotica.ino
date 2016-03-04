#line 1 "pinza_robotica_2.ino"
/********************************************************
 * Progetto Pinza Robotica - un esperimento del gruppo robottini di Raspibo 
 ********************************************************/

#include <PID_v1.h>
//    Dichirazione variabili PID

// Le variabili _p si riferiscono al valore posizione assoluta campionati dal sensore di hall
#include "Arduino.h"
void debug();
void setup();
void loop();
#line 9
double Setpoint_p, Input_p, Output_p;
// Le variabili _f si riferisco alla forza esercitata dalla pinza leggendo il trimmer
double Setpoint_f, Input_f, Output_f;
// Variabili tuning per processo PID http://playground.arduino.cc/Code/PIDLibrary
double consKp_p=1, consKi_p=0.08, consKd_p=0;
double consKp_f=2, consKi_f=0.5, consKd_f=0.02;
//Specify the links and initial tuning parameters
PID PID_p(&Input_p, &Output_p, &Setpoint_p, consKp_p, consKi_p, consKd_p, DIRECT);
//Specify the links and initial tuning parameters
PID PID_f(&Input_f, &Output_f, &Setpoint_f, consKp_f, consKi_f, consKd_f, DIRECT);

//      Dichiarazione variabili pin Arduino 

//MOT12 indica le variabili usate per pilotare il ponte H 1,2 dell'integrato L293D
//questo ponte pilota il movimento della pinza
//il ponte viene pilotato con un segnale pwm sull'enable per regolare velocit\u00e0 e forza del motore
#define MOT_12_PWM 9
//la direzione viene pilotata con un segnale digitale che commuta il segnale tra i pin 1A e 2A del ponte
#define MOT_12_DIR 4     
//MOT34 indica le variabili usate per pilotare il ponte H 3,4 dell'integrato L293D
//questo ponte pilote il movimento di torsione del polso
//il ponte viene pilotato con un segnale pwm sull'enable per regolare velocità e forza del motore
#define MOT_34_PWM 10    
//la direzione viene pilotata con un segnale digitale che commuta il segnale tra i pin 3A e 4A del ponte
#define MOT_34_DIR 5     
//Sensore hall analogico, rileva la posizione assoluta della griffa
#define P_POS A0 
//Sensore resistivo (trimmer), rileva la forza esercitata dalla griffa misurando la compressione di una molla all'interno della frizione meccanica
#define P_FORZA A1
//Asse X joystick usato per pilotare la pinza (valori 0-1023 posizione a riposo 512)
#define P_SETPT_PINZA A7
//Asse y joystick usato per pilotare rotazione del polso (valori 0-1023 posizione a riposo 512)
#define P_SETPT_ROT A6
//Bottone joystick usato per memorizzare la posizione della pinza
#define P_OLD 6
#define Led_R 13

//   Dichiarazione variabili    
#define DEBUG_INTERVAL 1000
#define OffSet_Joy 505
#define OffSet_Forza 510
#define InMax 255
#define InMin 1
//#define Forza_Max 255



unsigned int pinza_pwm=0;            //Variabile di appoggio per pilotaggio segnale pwm
boolean pinza_dir=0;        //Variabile di appoggio per pilotaggio direzione
int polso_pwm=0;            //Variabile di appoggio per pilotaggio segnale pwm
boolean polso_dir=0;        //Variabile di appoggio per pilotaggio direzione
int serial_inc_byte = 0;    //byte in ingresso da porta seriale pc usata per pilotaggio in alternativa al joystick
unsigned long time;         //in questa var viene salvato il valore in millis (tempo trascorso dal boot) per stampa debug ogni secondo
//Intervallo di tempo tra la scrittura delle info di debug in millisecondi



int incomingByte   = 0;      // for incoming serial data
int Forza = 0;
int Joy = 0;
int Mem_Joy=0;
int Forza_Max= 255;
boolean Click_On =0;
boolean Mem_Click =0;

void debug() {
// Serial.print(pinza_pwm); 
// Serial.print(";"); 
Serial.print("pinza_dir ");
if (digitalRead(MOT_34_DIR)){Serial.print("1");}
else{Serial.print("0");}
//Serial.print(" - MEM");
//if (Click_On){Serial.print("1");}
//else{Serial.print("0");}

 Serial.print(" - ForzaMax;"); 
 Serial.print(Forza_Max);
// Serial.prin(";");  
//Setpoint_f=analogRead(P_SETPT_PINZA);
 Serial.print(" - Setpoint: ");
 Serial.print(Setpoint_f);
 Serial.print(" - Input: ");
 Serial.print(Input_f);
 Serial.print(" - pinza_pwm: ");
 Serial.print(pinza_pwm);
 
 //Serial.print(" - Out: ");
 //Serial.print(Output_f);
 Serial.println(" ");
} 

void setup()
{
  Serial.begin(115200);
  Serial.println ("Pinza robotica project");
  pinMode(MOT_12_PWM, OUTPUT);
  pinMode(MOT_12_DIR, OUTPUT);
  pinMode(MOT_34_PWM, OUTPUT);
  pinMode(MOT_34_DIR, OUTPUT); 
  pinMode(Led_R, OUTPUT);
  pinMode(P_OLD, INPUT_PULLUP);
  time=millis();
  //registri TMR1 per pilotare i PWM a 16 MHz
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS12);
 
  //OCR2B = 50;
  PID_f.SetMode(AUTOMATIC);
  PID_f.SetOutputLimits(-255, 255);
  PID_f.SetSampleTime(10);
}

void loop()
{
 //Ricezione comandi seriali per modificare delle variabili
 if (Serial.available() > 0) {
   incomingByte = Serial.read();
    switch(incomingByte) {
      case '+':                   //Modifica della pressione di presa pinza 
        if (Forza_Max<InMax) {
          Forza_Max = Forza_Max+5;
          }
        break;
      case '-': 
        if (Forza_Max>InMin) {
          Forza_Max = Forza_Max-5;
          
        }        
        break;
     }
  }
  if (millis()>time+1000) { debug(); time=millis(); } // Qui viane lanciato il comando per scrivere le variabili di DEBUG
  
  Forza = analogRead(P_FORZA);            // Leggo la compressione della molla   
  Forza = Forza- OffSet_Forza;            // L'OffSet va tarato in base alla posizione de potenziomentro, molla a riposo = OffSet 
  
  
  // Qui abilito con il click del Joysitck il mantenimento di una posizione specifica, quando il led rosso è acceso la posizione viene mantenuta
  if((digitalRead(P_OLD)==0)&(Click_On==0)&(Mem_Click==0)){
    Mem_Click=1;
    Click_On=1;
    digitalWrite(Led_R,HIGH);
    }
  if((digitalRead(P_OLD)==0)&(Click_On==1)&(Mem_Click==0)){
    Mem_Click=1;
    Click_On=0;
    digitalWrite(Led_R,LOW);
    }
  if((digitalRead(P_OLD)==1)&(Mem_Click==1)){
    Mem_Click=0;
    }
    
  if(Click_On==0){
    
    Joy = analogRead(P_SETPT_PINZA); //Leggo la compressione della pinza
    Joy = Joy-OffSet_Joy;
    if(Joy>Forza_Max){Joy=Forza_Max;}
    if(Joy<-255){Joy=-255;}
    
  }

  
  
  // Aggiorno i valori del pid per l'elaborazione
  Setpoint_f =Joy; 
  Input_f = Forza;
  PID_f.Compute();
  // In base alla risposta del pid decido la direzione ed aggiorno il PWM con l'assoluto intero 
  pinza_pwm=Output_f;
  if(Output_f>0){digitalWrite(MOT_12_DIR,LOW);}
    else{digitalWrite(MOT_12_DIR,HIGH);}
  if(pinza_pwm>255){pinza_pwm=65536-pinza_pwm;}
  if(pinza_pwm>255){pinza_pwm=255;}
  //analogWrite(MOT_34_PWM,pinza_pwm);
  OCR1A = pinza_pwm;

}
