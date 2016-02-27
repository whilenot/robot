/*
 * wheels.c
 */

#include "Arduino.h"
#include "robot.h"
#include "wheels.h"

static const int DirA   = 12;
static const int DirB   = 13;
static const int SpeedA =  3;
static const int SpeedB = 11;
static const int BrakeA =  9;
static const int BrakeB =  8;


/* Initialization */
void initWheels(){
  /* Channel A */
  pinMode(DirA, OUTPUT);       // Direction pin as output
  pinMode(BrakeA, OUTPUT);     // Brake pin as output
  digitalWrite(DirA, LOW);     // Set forward direction 
  digitalWrite(BrakeA, HIGH);  // Engage brake

  /* Channel B */
  pinMode(DirB, OUTPUT);       // Direction pin as output
  pinMode(BrakeB, OUTPUT);     // Brake pin as output
  digitalWrite(DirB, HIGH);    // Set forward direction 
  digitalWrite(BrakeB, HIGH);  // Engage brake
}

/* Toggle brake ON or OFF */
void brake(char ch, int val) {
  switch(ch) {
    case RIGHT :
      digitalWrite(BrakeA, val);
      break;
    case LEFT :
      digitalWrite(BrakeB, val);
  }
}

/* Toggle both breaks ON or OFF */
void dualBreak(int val) {
  brake(LEFT, val);
  brake(RIGHT, val);
}

/* Toggle FORWARD/BACKWARD direction */
void dir(char ch, int val) {
  switch(ch) {
    case RIGHT :
      digitalWrite(DirA, !val);
      break;
    case LEFT :
      digitalWrite(DirB, val);
  }
}

/* Set wheel speed */
void speed(char ch, int val) {
  switch(ch) {
    case RIGHT :
      analogWrite(SpeedA, val);
      break;
    case LEFT :
      analogWrite(SpeedB, val);
  }
}
