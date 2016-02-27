/*
 * main.ino
 */

#include "robot.h"
#include "wheels.h"


/* Initialization */
void setup() {
  initWheels();
}

/* Main loop */
void loop() {
  /* Two second startup delay */
  delay(2000);
  
  /* Set the speed of the motors */
  speed(LEFT, 115);   // Set 45% duty cycle
  speed(RIGHT, 127);  // Set 50% duty cycle
  
  /* Drive forward for 5 seconds */
  dualBreak(OFF);    // Disengage both breaks
  delay(5000);
  
  /* Stop */
  dualBreak(ON);     // Engage both breaks
  delay(1000);

  /* Make a right turn */
  dir(RIGHT, BACKWARD);  // Drive right wheel backward
  dualBreak(OFF);
  delay(450);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Drive forward for 2 seconds */
  dir(RIGHT, FORWARD);   // Drive right wheel forward
  dualBreak(OFF);
  delay(2000);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Make a right turn */
  dir(RIGHT, BACKWARD);
  dualBreak(OFF);  
  delay(450);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Drive forward for 5 seconds */
  dir(RIGHT, FORWARD);
  dualBreak(OFF);
  delay(5000);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Make a right turn */
  dir(RIGHT, BACKWARD);
  dualBreak(OFF);
  delay(450);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Drive forward for 2 seconds */
  dir(RIGHT, FORWARD);
  dualBreak(OFF);
  delay(2000);

  /* Stop */
  dualBreak(ON);
  delay(1000);

  /* Make a right turn */
  dir(RIGHT, BACKWARD);
  dualBreak(OFF);
  delay(450);

  /* Stop */
  dualBreak(ON);
    
  while(1);
}
