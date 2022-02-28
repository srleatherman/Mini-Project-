#include "Encoder.h"
#define outputA 2
#define outputB 5
#define motorVoltage 9
#define button 4
#define signOfVoltage 7
#define flagIndicator 12

Encoder myEnc(outputA, outputB);
float controlSignal;
float desiredVoltage;
float desiredVoltagePWM;
float Kp = 5;
float Ki = 0;
int overallTime; 
double angularVelocity;
int time_elapsed = 0;
int startTime=0;
int startPosition;
int currentCounts;
float currentAngle;
float startAngle;
int duration;
int FIXED_CYCLE_TIME = 10; // unit is in ms
float pi = 3.1415; // known constant
int cpr = 3200; // known from the motor data sheet
float desiredAngle;
String data;
bool DataRead;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   
  pinMode(button, OUTPUT);
  digitalWrite(button, HIGH);
  
  pinMode(signOfVoltage,OUTPUT);
  digitalWrite(signOfVoltage, HIGH);
  
  pinMode(motorVoltage, OUTPUT);
  //analogWrite(motorVoltage, 0);
  
  pinMode(flagIndicator, INPUT);
  
  //startPosition = myEnc.read(); // set the start positon equal to 0
 
}

void loop() {
  // put your main code here, to run repeatedly:
  startTime = millis();
  
  switchFunction(); 
  
  currentCounts = myEnc.read(); // 
  //Serial.print(currentCounts);
  //Serial.print('\t');
  //the equation for angularVelocity is final angle - start angle/ time
  //the equation angle from counts is counts*2*pi/counts per rotation
  //FIXED_CYCLE_TIME is divided by 1000 because I need to change ms to seconds.
  //the final units are rads/sec
  currentAngle = (float)currentCounts*2*pi/(float)cpr;
  //Serial.print(currentAngle);
  //startAngle = (float)startPosition*2*pi/(float)cpr;
  angularVelocity = (float)1000*(currentAngle-startAngle)/((float)(startTime - duration));
  controller(desiredAngle, currentAngle, angularVelocity);
  //Serial.print(desiredAngle - currentAngle);
  //Serial.println('\t');
  //startPosition = currentAngle;
  duration = startTime;// sets the start positon to the finish position for the next iteration
  overallTime = millis();
  
  //Serial.print(overallTime);
  //Serial.print('\t');
  //Serial.print(currentAngle);
  //Serial.print('\t');
  //Serial.println(angularVelocity);
    
  //delay(FIXED_CYCLE_TIME - (overallTime - startTime));
  delay(10);
  if ((overallTime - startTime) > FIXED_CYCLE_TIME){
    //Serial.println("ERROR: the main function takes longer than the desired sampling rate");  
  }
}
void controller(float desiredAngle,float currentAngle, float angularVelocity){
  desiredVoltage = (float)Kp*(desiredAngle - currentAngle);
  //Serial.print(desiredVoltage);
  //Serial.print('\t');
  desiredVoltagePWM = (float)desiredVoltage*255/7.5;
  //Serial.print(desiredVoltagePWM);
  //Serial.print('\t');
  //Serial.println(desiredVoltagePWM);
  if( desiredVoltagePWM <0){
    digitalWrite(signOfVoltage,false);
  }else{
    digitalWrite(signOfVoltage, true);
  } 
  controlSignal = abs(desiredVoltagePWM);
  if(controlSignal >255){
    controlSignal = 255;
  }
    
  analogWrite(motorVoltage, controlSignal);
  startPosition = currentAngle;
}
void serialEvent(){ // waits for a input from the usb

if (Serial.available() > 0) {

  data = Serial.readStringUntil('\n'); //sets the sent data as a variable

  DataRead = true;

}

Serial.flush(); //clears the data

}
void switchFunction() { 
     if(data == "1"){
       Serial.println("1");
       desiredAngle =  (2*pi);
  
    }
    if(data == "2"){
      Serial.println("2");
      desiredAngle =  ((3*pi)/2);
  
    }
    if(data == "3"){
      Serial.println("3");
      desiredAngle =  (pi); 
  
    }
  
    if(data == "4"){
      Serial.println("4");
      desiredAngle =  (pi/2);
  
    }
  
    else {
      Serial.println("none");
      desiredAngle = (2*pi);
  
    }
  }
