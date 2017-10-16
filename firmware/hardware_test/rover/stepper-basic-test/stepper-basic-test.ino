///////////////////////////////////////////////////////////////////////////////////////
//   Test for driving both stepper motors of the Balancing Robot
//
//   This routine is designed to test coordination of two NEMA14 stepper motors
//   using DRV8825 stepper motor drivers from Pololu.  These drivers are based on
//   the TI DRV8825 IC: https://www.pololu.com/file/0J590/drv8825.pdf
//
//   An Arduino library for driving the motors can be found at:
//   https://github.com/laurb9/StepperDriver
//
//
///////////////////////////////////////////////////////////////////////////////////////
// Wiring Configuration for the Build,RVA Version
// Arduinio Pro Mini
//
// Arduino                      Motor Driver
// D2                           Left Step Pulse
// D3                           Left Direction
// D4                           Right Step Pulse
// D5                           Right Direction
//
// DRV8825 "positive" current is from AOUT1 -> AOUT2, BOUT1 -> BOUT2
// On the boardj this is  A1 -> A2, B1 -> B2
///////////////////////////////////////////////////////////////////////////////////////

#include <printf.h>
#include "DRV8825.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// right motor
#define DIR_RIGHT 3
#define STEP_RIGHT 2

// left motor
#define DIR_LEFT 5
#define STEP_LEFT 4

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 4

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
DRV8825 stepperLeft(MOTOR_STEPS, DIR_LEFT, STEP_LEFT);
DRV8825 stepperRight(MOTOR_STEPS, DIR_RIGHT, STEP_RIGHT);


SyncDriver controller(stepperLeft, stepperRight);

void setup()
{
  Serial.begin(9600);
  // Set target motor RPM to 1RPM
    stepperLeft.begin(10, MICROSTEPS);
    stepperRight.begin(10, MICROSTEPS);
 }

void loop() {
    
    controller.rotate(360, 360);  // rotate degrees on stepperLeft and stepperRight forward
    delay(1000);
    controller.rotate(-360, -360);  // rotate degrees on stepperLeft and stepperRight backward
    delay(30000);
}
  















