/*
 * wheels.cpp
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
Wheels::Wheels()
{
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

/* Toggle the brake of one motor or both ON or OFF */
void Wheels::toggleBrake(char ch, int val)
{
  switch(ch) {
    case RIGHT :
      digitalWrite(BrakeA, val);
      break;
    case BOTH :
      digitalWrite(BrakeA, val);
    case LEFT :
      digitalWrite(BrakeB, val);
  }
}

/* Set the direction of one motor FORWARD or BACKWARD */
void Wheels::setDirection(char ch, int val)
{
  switch(ch) {
    case RIGHT :
      digitalWrite(DirA, !val);
      break;
    case LEFT :
      digitalWrite(DirB, val);
  }
}

/* Set the speed of one motor */
void Wheels::setSpeed(char ch, int val) 
{
  switch(ch) {
    case RIGHT :
      analogWrite(SpeedA, val);
      break;
    case LEFT :
      analogWrite(SpeedB, val);
  }
}
