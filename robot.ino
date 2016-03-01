/*
 * robot.ino
 */

#include "robot.h"
#include "wheels.h"

/* Initialization */
void setup(){}

/* Main loop */
void loop() 
{
  Wheels wheels;      // Define wheel object
  
  delay(2000);        // Two second startup delay
  
  /* Set the wheels.setSpeed of the motors */
  wheels.setSpeed(LEFT, 115);   // Set 45% duty cycle
  wheels.setSpeed(RIGHT, 127);  // Set 50% duty cycle
  
  /* Drive forward for 5 seconds */
  wheels.toggleBrake(BOTH, OFF);    // Disengage both Brakes
  delay(5000);
  
  /* Stop */
  wheels.toggleBrake(BOTH, ON);     // Engage both Brakes
  delay(1000);

  /* Make a right turn */
  wheels.setDirection(RIGHT, BACKWARD);  // Drive right wheel backward
  wheels.toggleBrake(BOTH, OFF);
  delay(450);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Drive forward for 2 seconds */
  wheels.setDirection(RIGHT, FORWARD);   // Drive right wheel forward
  wheels.toggleBrake(BOTH, OFF);
  delay(2000);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Make a right turn */
  wheels.setDirection(RIGHT, BACKWARD);
  wheels.toggleBrake(BOTH, OFF);  
  delay(450);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Drive forward for 5 seconds */
  wheels.setDirection(RIGHT, FORWARD);
  wheels.toggleBrake(BOTH, OFF);
  delay(5000);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Make a right turn */
  wheels.setDirection(RIGHT, BACKWARD);
  wheels.toggleBrake(BOTH, OFF);
  delay(450);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Drive forward for 2 seconds */
  wheels.setDirection(RIGHT, FORWARD);
  wheels.toggleBrake(BOTH, OFF);
  delay(2000);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
  delay(1000);

  /* Make a right turn */
  wheels.setDirection(RIGHT, BACKWARD);
  wheels.toggleBrake(BOTH, OFF);
  delay(450);

  /* Stop */
  wheels.toggleBrake(BOTH, ON);
    
  while(1);
}
