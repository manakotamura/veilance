#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int inputPin = 2;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected ("sensorFlag") boolean
int val = 0;                    // variable for reading the sensor 
int iCounter = 0;               // counts iterations

Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

// Connect one stepper with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(200, 1);

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the bottom shield
Adafruit_StepperMotor *myStepper2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *myStepper3 = AFMSbot.getStepper(200, 2);


void forwardstep1() {  
  myStepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, SINGLE);
}

void releasestep1() {
  myStepper1->release();
}

// wrappers for the second motor!
void forwardstep2() {  
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, DOUBLE);
}
void releasestep2() {
  myStepper2->release();
}

// wrappers for the third motor!
void forwardstep3() {  
  myStepper3->onestep(FORWARD, INTERLEAVE);
}
void backwardstep3() {  
  myStepper3->onestep(BACKWARD, INTERLEAVE);
}
void releasestep3() {
  myStepper3->release();
}


// Now we'll wrap the 3 steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);

void setup() {
  pinMode(myStepper1, OUTPUT);     // declare stepper as output
  pinMode(myStepper2, OUTPUT);     // declare stepper as output
  pinMode(myStepper3, OUTPUT);     // declare stepper as output
  pinMode(inputPin, INPUT);     // declare sensor as input
  
  Serial.begin(9600);
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
   
  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(150.0);
  stepper1.moveTo(500);
    
  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(50.0);
  stepper2.moveTo(500);

  stepper3.setMaxSpeed(700.0);
  stepper3.setAcceleration(100.0);
  stepper3.moveTo(500);
}
 
void loop(){
  val = digitalRead(inputPin);  // read input value

  if (val == HIGH) {                    // check if the Sensor input is HIGH
    if (pirState == LOW) {              // RETRIGGERING - the beginning pirState, we have just turned on
      Serial.println("Motion detected!"); //We only want to print on the output change, not sensor state
      pirState = HIGH;                  // Change pirState to HIGH to turn sensor numb for a while
        digitalWrite(forwardstep1, HIGH);   // turn motors ON, changed from "stepper1.run()"
        digitalWrite(forwardstep2, HIGH);   // turn motors ON, changed from "stepper2.run()"
        digitalWrite(forwardstep3, HIGH);   // turn motors ON, changed from "stepper3.run()"
      iCounter = 0;                         // in the beginning, the number of iterations is 0
      }
  }else {                                 // if Sensor input is LOW and
    if (pirState == HIGH){                // if the sensor is numb
      Serial.println("Motion ended!");    // We only want to print on the output change, not sensor state
      pirState = LOW;                     // make state go back to the beginning, i.e. check sensor input
    }
  }
  iCounter++;                            // next iteration
                                         // add if statements for following sequences
 }
}


    // Change direction at the limits
//    if (stepper1.distanceToGo() == 0)
//  stepper1.moveTo(-stepper1.currentPosition());
//
//    if (stepper2.distanceToGo() == 0)
//  stepper2.moveTo(-stepper2.currentPosition());
//
//    if (stepper3.distanceToGo() == 0)
//  stepper3.moveTo(-stepper3.currentPosition());

//    stepper1.run();
//    stepper2.run();
//    stepper3.run();



