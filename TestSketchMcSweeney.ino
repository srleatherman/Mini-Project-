#include "Encoder.h" //this gives us access to the encoder.h files 
#define outputA 2 //this pin is output A for the encoder
#define outputB 5 // this pin is output B for the encoder
#define motorVoltage 9 //this is one of the two voltage pwm pins we could have picked
#define button 4 // this pin is for setting the interupt
#define signOfVoltage 7 // this is what direction the wheel will turn 
#define flagIndicator 12 // this pin is if there is a flag

//the following line of code sets up the encoder to read from outputs a and b and will be used throughout the code
//using the encoder.h file allows us to simplify the code
Encoder myEnc(outputA, outputB);

float controlSignal; //this variable is used in controller function
float desiredVoltage; //this is the desired voltage not in PWM
float desiredVoltagePWM; //this is the desired voltage in PWM
float Kp = 2; // design value based on simulation
float Ki = 0; // design value based on simulation
int overallTime; // this keeps track of the overall time it takes the loop to complete
int startTime=0; // this is the time the loop starts
int startPosition; // this is the start position of the encoder at the beginning of the loop
int currentCounts; // this is the counts that the encoder is currently at in the loop
float currentAngle; // this the angle that the encoder is currently at in the loop
float startAngle; // this is the starting angle that the encoder is at the beginning of the loop
int FIXED_CYCLE_TIME = 10; // unit is in ms
float pi = 3.1415; // known constant
int cpr = 3200; // counts per rotation known from the motor data sheet
float desiredAngle; // stores what the input from the PI means  
String data; // input from the PI
bool DataRead; // makes sure that the data read from the PI is being sent correctly


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //set up all of my output pins 
  pinMode(button, OUTPUT);
  digitalWrite(button, HIGH);
  
  pinMode(signOfVoltage,OUTPUT);
  digitalWrite(signOfVoltage, HIGH);
  
  pinMode(motorVoltage, OUTPUT);

  //set all of my input pins
  pinMode(flagIndicator, INPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  //reads time at begining of loop
  startTime = millis();

  //calls the function that determines what the desired angle is based on what is read in from the PI
  switchFunction(); 

  //reads the current state of the encoder in counts
  currentCounts = myEnc.read(); // this is reading the current counts from the encoder
  
  
  //the equation to get angle from counts is counts*2*pi/counts per rotation
  //the final units are radians
  currentAngle = (float)currentCounts*2*pi/(float)cpr;
  startAngle = (float)startPosition*2*pi/(float)cpr;
  
  controller(desiredAngle, currentAngle);
  
  //reads time at end of loop
  overallTime = millis();
  
  //delay is added to make sure that it is sampling the controller at a set rate    
  delay(FIXED_CYCLE_TIME);
  
  if ((overallTime - startTime) > FIXED_CYCLE_TIME){
    Serial.println("ERROR: the main function takes longer than the desired sampling rate");  
  }
}

//the contoller function implements the controller designed in the arduino code
void controller(float desiredAngle,float currentAngle){
  //the function takes in the desired angle and the current angle that the encoder is at

  //the desired voltage is determined by how large the difference between the desired and current angle
  //multiplied by the Kp value found. Since our KI value from our controller was zero that part is not implemented
  desiredVoltage = (float)Kp*(desiredAngle - currentAngle);

  //the desired boltage must be converted to a PWM value because that is what the motorVoltage can interpret
  desiredVoltagePWM = (float)desiredVoltage*255/7.5;

  //this checks the sign of the voltage
  if( desiredVoltagePWM <0){
    //if it is less than zero it is negative
    digitalWrite(signOfVoltage,false);
  }else{
    //if it is greater than zero it is positive
    digitalWrite(signOfVoltage, true);
  } 

  //this takes the unsigned PWM value
  controlSignal = abs(desiredVoltagePWM);

  //In the chance the the desired voltage exceeds 7.5 volts we can not meet those requirements, so we set
  //the voltage to the maximum possible PWM value of 255.
  if(controlSignal >255){
    controlSignal = 255;
  }
  //this writes to the pin that controls the voltage the voltage we derived.  
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
  //goes through a series of if statements to determine the desired angle based on the quadrants
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
