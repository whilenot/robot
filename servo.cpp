/************************************************************************/
/* servo.cpp - The .cpp file for servo management.                      */
/************************************************************************/

#include "Arduino.h"
#include "servo.h"
#include "robot.h"
#include <Servo.h>

// Arduino specific pins for servos.
#define LIFTING_ARM_SERVO_PIN       5
#define BUCKET_ROTATION_SERVO_PIN   48
#define CATAPULT_ARM_SERVO_PIN      2
#define CATAPULT_LOCKING_SERVO_PIN  4
#define TOP_SENSOR_SERVO_PIN        10

/// Servo min- and max angle values. Be careful here!
/// These values depend on the increment- and decrement amount specified in state.cpp
#define LIFTING_ARM_SERVO_MIN       0
#define LIFTING_ARM_SERVO_MAX       62

#define BUCKET_ROTATION_SERVO_MIN   0
#define BUCKET_ROTATION_SERVO_MAX   200

#define CATAPULT_ARM_SERVO_MIN      0
#define CATAPULT_ARM_SERVO_MAX      120

#define CATAPULT_LOCKING_SERVO_MIN  0
#define CATAPULT_LOCKING_SERVO_MAX  99

#define TOP_SENSOR_SERVO_MIN        15
#define TOP_SENSOR_SERVO_MAX        135

/// Initialization of array holding the servo objects.
/// The order of this array is defined in robot.h
static Servo servo[5];

/// Initialization of array holding the Arduino specific pins.
static const uint8_t servo_pin[5] = {
    LIFTING_ARM_SERVO_PIN,
    BUCKET_ROTATION_SERVO_PIN,
    CATAPULT_ARM_SERVO_PIN,
    CATAPULT_LOCKING_SERVO_PIN,
    TOP_SENSOR_SERVO_PIN
};

/// Initialization of array holding the servo MIN angle values.
static const uint8_t servo_min_angle[5] = {
    LIFTING_ARM_SERVO_MIN,
    BUCKET_ROTATION_SERVO_MIN,
    CATAPULT_ARM_SERVO_MIN,
    CATAPULT_LOCKING_SERVO_MIN,
    TOP_SENSOR_SERVO_MIN
};

/// Initialization of array holding the servo MAX angle values.
static const uint8_t servo_max_angle[5] = {
    LIFTING_ARM_SERVO_MAX,
    BUCKET_ROTATION_SERVO_MAX,
    CATAPULT_ARM_SERVO_MAX,
    CATAPULT_LOCKING_SERVO_MAX,
    TOP_SENSOR_SERVO_MAX
};

/// Initialization of array holding the current angle of the servos.
/// Assumes startup states to be implemented. That's why not all values are == 0.
static int16_t servo_angle[5] = {
    41,  // Lifting arm servo
    42,  // Bucket rotation servo
    41,  // Catapult arm servo
    42,  // Catapult locking servo
    -15  // Top sensor servo
};

/************************************************************************/
/* Attach desired servo (@param _servo).                                */
/************************************************************************/
void servo_attach(uint8_t _servo)
{
    servo[_servo].attach(servo_pin[_servo]);
}

/************************************************************************/
/* Detach desired servo (@param _servo).                                */
/************************************************************************/
void servo_detach(uint8_t _servo)
{
    servo[_servo].detach();
}

/************************************************************************/
/* Decrements angle amount (@param amount) of one servo (@param _servo).*/
/************************************************************************/
void servo_angle_decrement(uint8_t _servo, uint8_t amount)
{
    /// Decrements angle.
    servo_angle[_servo] -= amount;
    
    /// Rotates the servo to desired angle.
    servo[_servo].write(servo_angle[_servo]);
}

/************************************************************************/
/* Increments angle amount (@param amount) of one servo (@param _servo).*/
/************************************************************************/
void servo_angle_increment(uint8_t _servo, uint8_t amount)
{
    /// Increments angle.
    servo_angle[_servo] += amount;
    
    /// Rotates the servo to desired angle.
    servo[_servo].write(servo_angle[_servo]);    
}

/************************************************************************/
/* @returns whether desired servo (@param servo) is at MIN angle or not.*/
/************************************************************************/
bool servo_at_min_angle(uint8_t servo)
{
    return (servo_angle[servo] == servo_min_angle[servo]) ? true : false;
}

/************************************************************************/
/* @returns whether desired servo (@param servo) is at MAX angle or not.*/
/************************************************************************/
bool servo_at_max_angle(uint8_t servo)
{
    return (servo_angle[servo] == servo_max_angle[servo]) ? true : false;
}

/************************************************************************/
/* Rotation of the top sensor servo.                                    */
/************************************************************************/
void top_sensor_servo_rotate(void)
{
    static bool rotate_left = true;
    
    /// The top sensor servo rotates left.
    if (rotate_left) {
        /// Increments angle.
        servo_angle_increment(TOP_SENSOR, 30);
        
        if (servo_at_max_angle(TOP_SENSOR)) {
            rotate_left = false;
        }
        
    /// The top sensor servo rotates right.
    } else {
        /// Decrements angle.
        servo_angle_decrement(TOP_SENSOR, 30);
        
        if (servo_at_min_angle(TOP_SENSOR)) {
            rotate_left = true;
        }
    }
}

/************************************************************************/
/* Rotates the top sensor servo to mid position.                        */
/************************************************************************/
void top_sensor_servo_mid(void)
{
    servo[TOP_SENSOR].write(TOP_SENSOR_SERVO_MIN + ((TOP_SENSOR_SERVO_MAX - TOP_SENSOR_SERVO_MIN) / 2));
}

/************************************************************************/
/* @returns the value of the top servo angle.                           */
/************************************************************************/
bool top_servo_at_mid(void)
{
    return (servo_angle[TOP_SENSOR] == (TOP_SENSOR_SERVO_MIN + ((TOP_SENSOR_SERVO_MAX - TOP_SENSOR_SERVO_MIN) / 2))) ? true : false;
}

/************************************************************************/
/* @returns whether the top servo angle is on the right side or not.    */
/************************************************************************/
bool top_servo_right_angle(void)
{
    return (servo_angle[TOP_SENSOR] <= ((TOP_SENSOR_SERVO_MAX - TOP_SENSOR_SERVO_MIN) / 2)) ? true : false;
}