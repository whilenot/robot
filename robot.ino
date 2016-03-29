/*
   robot.ino
*/

#include "encoders.h"
#include "robot.h"
#include "wheels.h"

int wheelSpeedLeft  = 100;
int wheelSpeedRight = 114;

/* Initialization of the robot */
void setup()
{
  Serial.begin(9600);

  /* Initialize the wheels */
  wheels_init();

  /* Set the speed of the motors */
  wheels_setSpeed(LEFT,  wheelSpeedLeft);
  wheels_setSpeed(RIGHT, wheelSpeedRight);
  
  /* Two second startup delay */
  delay(2000);

  /* Initialize the encoders */
  encoders_init();
  
  /* Toggle brakes ON or OFF */
  wheels_toggleBrake(BOTH, ON);
}

/* Main loop */
void loop()
{
  
}
