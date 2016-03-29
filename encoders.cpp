/*
 * encoders.cpp
 */

#include "Arduino.h"
#include "robot.h"
#include "timers.h"
#include "wheels.h"
#include <PID_v1.h>

static const int LeftSensor  = 19;
static const int RightSensor = 20;

volatile int counterLeft  = 0;
volatile int counterRight = 0;
volatile int startCounter = 0;

double error;
double output;
double setpoint = 0;

double Kp = 2.5;
double Ki = 0;
double Kd = 0;

PID wheelControl(&error, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/* Interrupt function for left encoder */
void countLeft()
{
  counterLeft++;
}

/* Interrupt function for right encoder */
void countRight()
{
  counterRight++;
}

/* Initialization of encoders */
void encoders_init(void)
{
  /* Arduino specific initialization */
  pinMode(LeftSensor,  INPUT);
  pinMode(RightSensor, INPUT);
  digitalWrite(LeftSensor,  HIGH);
  digitalWrite(RightSensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(LeftSensor),  countLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightSensor), countRight, CHANGE);

  /* Initialization of wheel control */
  wheelControl.SetMode(AUTOMATIC);
  wheelControl.SetOutputLimits(-128, 127);

  /* Initialization of Timer4 */
  timer4_init();
}

/* Interrupt service routine for wheel control */
ISR(TIMER4_COMPA_vect)
{
  Serial.print("L: ");
  Serial.print(counterLeft);
  Serial.print(", R: ");
  Serial.print(counterRight);

  if (startCounter >= 2) {
    error = counterRight - counterLeft;
    
    Serial.print(", E: ");
    Serial.print(error);
    
    if(error != 0) {
      wheelControl.Compute();

      wheels_setSpeed(RIGHT, wheelSpeedRight + (int) output);

      Serial.print(", O: ");
      Serial.println(output);
    } else {
      Serial.print(", O: ");
      Serial.println(0.0);
    }
  } else {
    Serial.println();
    startCounter++;
  }
  
  counterLeft  = 0;
  counterRight = 0;
}
