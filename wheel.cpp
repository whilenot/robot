/************************************************************************/
/* wheel.cpp - The .cpp file for controlling the wheels of the robot.   */
/************************************************************************/

#include "Arduino.h"
#include "wheel.h"
#include "robot.h"

/// Arduino specific pins for using the motor shield.
#define BRAKE_A_PIN  9
#define BRAKE_B_PIN  8
#define DIR_A_PIN    12
#define DIR_B_PIN    13
#define SPEED_A_PIN  3
#define SPEED_B_PIN  11

/// Wheel speed in High Speed Mode.
#define WHEEL_SPEED_RIGHT_HIGH  120
#define WHEEL_SPEED_LEFT_HIGH   120

/// Wheel speed in Low Speed Mode.
#define WHEEL_SPEED_RIGHT_LOW   85
#define WHEEL_SPEED_LEFT_LOW    85

/************************************************************************/
/* Initialization of the wheel control.                                 */
/************************************************************************/
void wheel_init(void)
{
    /* Channel A -- Right wheel */
    pinMode(DIR_A_PIN, OUTPUT);       // Direction pin as output
    pinMode(BRAKE_A_PIN, OUTPUT);     // Brake pin as output
    digitalWrite(DIR_A_PIN, LOW);     // Set forward direction 
    digitalWrite(BRAKE_A_PIN, HIGH);  // Engage brake
    analogWrite(SPEED_A_PIN, WHEEL_SPEED_RIGHT_HIGH); // Set wheel speed
    

    /* Channel B -- Left Wheel */
    pinMode(DIR_B_PIN, OUTPUT);       // Direction pin as output
    pinMode(BRAKE_B_PIN, OUTPUT);     // Brake pin as output
    digitalWrite(DIR_B_PIN, HIGH);    // Set forward direction 
    digitalWrite(BRAKE_B_PIN, HIGH);  // Engage brake
    analogWrite(SPEED_B_PIN, WHEEL_SPEED_LEFT_HIGH); // Set wheel speed
}

/************************************************************************/
/* Toggle the brake pin for one motor, or both.                         */
/************************************************************************/
void wheel_toggle_brake(uint8_t wheel, uint8_t value)
{
    switch(wheel) {
        case RIGHT :
            digitalWrite(BRAKE_A_PIN, value);
            break;
        case BOTH :
            digitalWrite(BRAKE_A_PIN, value);
        case LEFT :
            digitalWrite(BRAKE_B_PIN, value);
            break;
    }
}

/************************************************************************/
/* Set the direction of one motor, or both.                             */
/************************************************************************/
void wheel_set_direction(uint8_t wheel, uint8_t value)
{
    switch(wheel) {
        case RIGHT :
            digitalWrite(DIR_A_PIN, !value);
            break;
        case BOTH :
            digitalWrite(DIR_A_PIN, !value);
        case LEFT :
            digitalWrite(DIR_B_PIN, value);
            break;
    }
}

/************************************************************************/
/* Set the motors in High- or Low Speed Mode.                           */
/************************************************************************/
void wheel_set_speed(uint8_t value)
{
    /// High Speed Mode
    if (value) {
        analogWrite(SPEED_A_PIN, WHEEL_SPEED_RIGHT_HIGH);
        analogWrite(SPEED_B_PIN, WHEEL_SPEED_LEFT_HIGH);
    /// Low Speed Mode
    } else {
        analogWrite(SPEED_A_PIN, WHEEL_SPEED_RIGHT_LOW);
        analogWrite(SPEED_B_PIN, WHEEL_SPEED_LEFT_LOW); 
    }
}
