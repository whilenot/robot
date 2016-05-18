/************************************************************************/
/* state.cpp - The .cpp file for state management.                      */
/************************************************************************/

#include "Arduino.h"
#include "state.h"
#include "compass.h"
#include "robot.h"
#include "sensor.h"
#include "wheel.h"
#include <Servo.h>

/// Arduino specific pins for servos.
#define LIFTING_ARM_SERVO_PIN       5
#define BUCKET_ROTATION_SERVO_PIN   48
#define CATAPULT_ARM_SERVO_PIN      2
#define CATAPULT_LOCKING_SERVO_PIN  4
#define TOP_SENSOR_SERVO_PIN        10

/// Servo max angle values. Be careful here!
#define LIFTING_ARM_SERVO_MAX       60
#define BUCKET_ROTATION_SERVO_MAX   200
#define CATAPULT_ARM_SERVO_MAX      120
#define CATAPULT_LOCKING_SERVO_MAX  60

/// Delay until the lifting arm moves down (in 5 ms resolution, so 200 => 1 second delay).
#define LIFTING_ARM_DELAY_TIME  200

/// Top servo min- and max angle value.
#define TOP_SENSOR_SERVO_MIN    15
#define TOP_SENSOR_SERVO_MAX    135

/// Initialization of servo objects.
Servo lifting_arm_servo;
Servo bucket_rotation_servo;
Servo catapult_arm_servo;
Servo catapult_locking_servo;
Servo top_sensor_servo;

/// Counter variable for the top servo angle.
static volatile uint8_t top_servo_angle_counter = TOP_SENSOR_SERVO_MIN;

/// Variable holding the current state of the robot.
static volatile int8_t state = -4;

/// Variables holding information of the number of balls (if any) that can be launched.
static volatile bool got_second_ball    = false;
static volatile bool prepared_to_launch = false;

/************************************************************************/
/* Rotate the lifting arm servo (states -4, 2, 3).                      */
/************************************************************************/
void lifting_arm_servo_rotate(uint8_t direction)
{
    static int servo_angle_counter = 41;
    
    switch(direction) {
        case DOWN :
            servo_angle_counter--;
            break;
        case UP :
            servo_angle_counter+=2;
            break;
    }
    
    if (servo_angle_counter < LIFTING_ARM_SERVO_MAX) {
        lifting_arm_servo.attach(LIFTING_ARM_SERVO_PIN);
        lifting_arm_servo.write(servo_angle_counter);
        
        if (servo_angle_counter == 0) {
            lifting_arm_servo.detach();
            state++;
        }
    } else {
        if (servo_angle_counter == LIFTING_ARM_SERVO_MAX + LIFTING_ARM_DELAY_TIME) {
            servo_angle_counter = LIFTING_ARM_SERVO_MAX;
            lifting_arm_servo.detach();
            state++;
        }
    }
}

/************************************************************************/
/* Rotate the bucket rotation servo (states -3, 1, 4).                  */
/************************************************************************/
void bucket_rotation_servo_rotate(uint8_t direction)
{
    static int servo_angle_counter = 42;
    
    switch(direction) {
        case OUT :
            servo_angle_counter-=2;
            break;
        case IN :
            servo_angle_counter+=4;
            break;
    }
    
    bucket_rotation_servo.attach(BUCKET_ROTATION_SERVO_PIN);
    bucket_rotation_servo.write(servo_angle_counter);
    
    if (servo_angle_counter == 0) {
        bucket_rotation_servo.detach();
        
        if (state == 4) {
            if (!got_second_ball) {
                state = 0;
            } else {
                got_second_ball = false;
                state = 8;
            }
        } else {
            state++;
        }        
    } else if (servo_angle_counter == BUCKET_ROTATION_SERVO_MAX) {
        bucket_rotation_servo.detach();
        
        if (bucket_sensor_triggered()) {
            if (!prepared_to_launch) {
                prepared_to_launch = true;
                state++;
            } else {
                got_second_ball = true;
                state = 0;
            }            
        } else {
            state = 4;
        }
    }
}

/************************************************************************/
/* Rotate the catapult arm servo (states -2, 9, 11).                    */
/************************************************************************/
void catapult_arm_servo_rotate(uint8_t direction)
{
    static int servo_angle_counter = 41;
    
    switch(direction) {
        case DOWN :
            servo_angle_counter--;
            break;
        case UP :
            servo_angle_counter++;
            break;
    }
    
    catapult_arm_servo.attach(CATAPULT_ARM_SERVO_PIN);
    catapult_arm_servo.write(servo_angle_counter);
    
    if (servo_angle_counter == 0) {
        catapult_arm_servo.detach();
        
        if (state == 11) {
            if(!got_second_ball) {
                state = 0;
            } else {
                state = 2;
            }
            
        } else {
            state++;
        }
    } else if (servo_angle_counter == CATAPULT_ARM_SERVO_MAX) {
        state++;        
    }
}

/************************************************************************/
/* Rotate the catapult locking servo (states -1, 8, 10).                */
/************************************************************************/
void catapult_locking_servo_rotate(uint8_t direction)
{
    static int servo_angle_counter = 48;
    
    switch(direction) {
        case UNLOCK :
            servo_angle_counter-=4;
            break;
        case LOCK :
            servo_angle_counter+=4;
            break;
    }
    
    catapult_locking_servo.attach(CATAPULT_LOCKING_SERVO_PIN);
    catapult_locking_servo.write(servo_angle_counter);
    
    if (servo_angle_counter == 0 || servo_angle_counter == CATAPULT_LOCKING_SERVO_MAX) {
        catapult_arm_servo.detach();
        catapult_locking_servo.detach();
        state++;
    }
}

/************************************************************************/
/* Default state (state 0).                                             */
/************************************************************************/
void default_state(void) 
{
    static int bucket_sensor_trig_counter = 0;
    static int delay_counter = 0;
    
    static bool triggered = false;
    
    if (!got_second_ball) {
        if (bucket_sensor_trig_counter == 3) {
            bucket_sensor_trig_counter = 0;
            triggered = false;
            state++;
        } else if (triggered) {
            delay_counter++;
        
            if (delay_counter == 100) {
                delay_counter = 0;
                triggered = false;
            }
        } else if (bucket_sensor_triggered()) {
            bucket_sensor_trig_counter++;
            triggered = true;
        } else {
            bucket_sensor_trig_counter = 0;
        }
    }    
    
    if (top_sensor_triggered()) {
        if (!prepared_to_launch) {
            if (bucket_sensor_trig_counter == 0) {
                //wheel_toggle_brake(BOTH, ON);
                wheel_set_speed(LOW);
                
                if (top_servo_angle_counter <= 60) {
                    /// Turn left
                    wheel_set_direction(LEFT, BACKWARD);
                } else {
                    /// Turn right
                    wheel_set_direction(RIGHT, BACKWARD);
                }
                
                state = 5;  // Ska vara 6
            }
        } else {
            prepared_to_launch = false;
            /// Turn left to prepare for launch (state 7)
            state = 8;
        }
    }
}

/************************************************************************/
/* Turn to mid wall state (state 5).                                    */
/************************************************************************/
void turn_to_mid_wall_state(void)
{
    if (compass_heading_triggered())
    {
        wheel_set_direction(BOTH, FORWARD);
        wheel_set_speed(HIGH);
        
        state = 0;
    }
}

/************************************************************************/
/* Turn for wall state (state 6).                                       */
/************************************************************************/
void turn_for_wall_state(void) {
    static int delay_counter = 0;
    
    delay_counter++;
    
    if (delay_counter == 100) {
        delay_counter = 0;
        
        wheel_set_direction(LEFT, FORWARD);
        wheel_set_direction(RIGHT, FORWARD);
        
        state = 0;
    }    
}

/************************************************************************/
/* Rotation of the top sensor servo. Allowed in state 0 and 1.          */
/************************************************************************/
void top_sensor_servo_rotate(void)
{
    static bool rotate_left = true;
    
    if (rotate_left) {
        top_servo_angle_counter+=15;
        
        if (top_servo_angle_counter == TOP_SENSOR_SERVO_MAX) {
            rotate_left = false;
        }
    } else {
        top_servo_angle_counter-=15;
        
        if (top_servo_angle_counter == TOP_SENSOR_SERVO_MIN) {
            rotate_left = true;
        }
    }
        
    top_sensor_servo.attach(TOP_SENSOR_SERVO_PIN);
    top_sensor_servo.write(top_servo_angle_counter);
    //top_sensor_servo.detach();
}

/************************************************************************/
/* Returns the current state of the system.                             */
/************************************************************************/
int8_t current_state(void)
{
    return state;
}
