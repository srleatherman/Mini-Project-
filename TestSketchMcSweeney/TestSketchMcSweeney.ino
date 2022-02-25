#include "Encoder.h"
#define outputA 2
#define outputB 7
#define motorVoltage 4
Encoder myEnc( outputA, outputB);
int overallTime= millis(); 
int angularVelocity;
int time_elapsed = 0;
int startTime=0;
int startPosition;
int finishPosition;
int FIXED_CYCLE_TIME = 10; // unit is in ms
int pi = 3.1415; // known constant
int cpr = 3200; // known from the motor data sheet


void setup() {
  // put your setup code here, to run once:
 Serial.begin(19200);
 
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(motorVoltage, OUTPUT);
  analogWrite(motorVoltage, LOW);
  //aLastState = digitalRead(outputA);
  overallTime = millis();
  myEnc.write(0); //set the encoder position to 0
  startPosition =0; // set the start positon equal to 0
}

void loop() {
  // put your main code here, to run repeatedly:
  startTime = millis();
 if(millis()<=FIXED_CYCLE_TIME+overallTime){
  finishPosition = myEnc.read(); // sets the final position equal to the current position
  //the equation for angularVelocity is final angle - start angle/ time
  //the equation angle from counts is counts*2*pi/counts per rotation
  //FIXED_CYCLE_TIME is divided by 1000 because I need to change ms to seconds.
  //the final units are rads/sec
  angularVelocity = (finishPosition*2*pi/(float)cpr -startPosition*2*pi/(float)cpr)/((float)FIXED_CYCLE_TIME/1000);
  startPosition = finishPosition; // sets the start positon to the finish position for the next iteration
 }
  overallTime = millis();
  
  //overallTime = millis();
  if(overallTime <= 2000){
    Serial.println(overallTime);
    Serial.println(angularVelocity);
    if(overallTime >1000){
      //set motor voltage to desired positve value
      analogWrite(motorVoltage, HIGH);
    }
  }
  
  if ((overallTime - startTime) > FIXED_CYCLE_TIME){
    Serial.println("ERROR: the main function takes longer than the desired sampling rate");  
  }
}
