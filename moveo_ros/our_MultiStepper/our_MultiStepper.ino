// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

////FORWARD////
//float s= 300.0;  // start motion +ve, return -ve
////RETURN////
float s= -0.10;  // start motion +ve, return -ve

float d1= s * 680000;                   ////640 step per 1mm
float d2= s * 8;
float d3= -s * 60;  //       /// from motor prespective, CW is -ve , CCW is +ve
float d4= s * 20;  //
float d5= -s * 10;
float d6= s * 10;  //
float d7= -s * 10;
float t= 1;
// Define some steppers and the pins they will use

///////////AccelStepper stepper1(1,stp pin,dir pin)///////////////
//j1
AccelStepper stepper1(1,2,32);
//j2
AccelStepper stepper2(1,3,34); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//AccelStepper stepper2(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
//AccelStepper stepper3(AccelStepper::FULL2WIRE, 10, 11);
//j3
AccelStepper stepper3(1,4,36);
//j4
AccelStepper stepper4(1,5,38);
//j5
AccelStepper stepper5(1,6,40);
//J6
AccelStepper stepper6(1,7,42);
//j7
AccelStepper stepper7(1,8,44);


// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup() {
  Serial.begin(9600);

  // Configure each stepper
    stepper1.setMaxSpeed(20000);
    stepper1.setAcceleration(10000);
    

    stepper2.setMaxSpeed(d2/t);
    stepper2.setAcceleration(10000);
   
   
    stepper3.setMaxSpeed(d3/t);
    stepper3.setAcceleration(10000);

    stepper4.setMaxSpeed(d4/t);
    stepper4.setAcceleration(10000);
 
    stepper5.setMaxSpeed(d5/t);
    stepper5.setAcceleration(10000);
 
//
    stepper6.setMaxSpeed(d6/t);
    stepper6.setAcceleration(10000);
   

    stepper7.setMaxSpeed(d7/t);
    stepper7.setAcceleration(10000);
  

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);
  steppers.addStepper(stepper7);
}

void loop() {
  long positions[7]; // Array of desired stepper positions
  
  positions[0] = d1;
  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;
  positions[4] = 0;
  positions[5] = 0;
  positions[6] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  
  // Move to a different coordinate
  
  positions[0] = d1;
  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;
  positions[4] = 0;
  positions[5] = 0;
  positions[6] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}
