// Blocking.pde
// -*- mode: C++ -*-
//
// Shows how to use the blocking call runToNewPosition
// Which sets a new target position and then waits until the stepper has 
// achieved it.
//
// Copyright (C) 2009 Mike McCauley
// $Id: Blocking.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper(1,28,46);

void setup()
{  
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(1000.0);
}

void loop()
{    
    stepper.runToNewPosition(0);
    stepper.runToNewPosition(1000);
    stepper.runToNewPosition(2000);
    stepper.runToNewPosition(1000);
}
