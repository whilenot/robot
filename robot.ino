/************************************************************************/
/* robot.ino - The main file for the robot project.                     */
/************************************************************************/

#include "robot.h"
#include "compass.h"
#include "sensor.h"
#include "servo.h"
#include "state.h"
#include "timer.h"
#include "wheel.h"

/// Variable saying when it is OK to update the compass heading.
/// This variable is polled in the main loop.
static volatile bool ok_for_compass = false;

/// Variable saying when it is OK for the bucket sensor to do a ping request.
/// This variable is polled in the main loop.
static volatile bool ok_for_bucket_sensor = false;

/// Variable saying when it is OK for the bucket sensor to do a ping request.
/// This variable is polled in the main loop.
static volatile bool ok_for_top_sensor = false;

/// Variable saying if new compass heading should be calculated or retrieved.
static volatile bool new_heading = true;

/************************************************************************/
/* Initialization of the robot.                                         */
/************************************************************************/
void setup()
{
    /// Start serial communication.
    Serial.begin(9600);

    /// Initialization of the compass.
    compass_init();
    
    /// Initialization of the wheels.
    wheel_init();
    
    /// Initialization of state management.
    state_init();
    
    /// Initialization of Timer4.
    timer4_init();
    
    /// Two second startup delay.
    delay(2000);
  
    /// Toggle brakes ON or OFF.
    wheel_toggle_brake(BOTH, OFF);
}

/************************************************************************/
/* Program main loop.                                                   */
/************************************************************************/
void loop()
{
    if (ok_for_bucket_sensor) {
        ok_for_bucket_sensor = false;
        bucket_sensor_update();
    }
    
    if (ok_for_top_sensor) {
        ok_for_top_sensor = false;
        top_sensor_update();
        top_sensor_servo_rotate();
    }
    
    if (ok_for_compass) {
        ok_for_compass = false;
        
        if(new_heading) {
            compass_update(NEW_HEADING);
        } else {
            compass_update(GET_HEADING);
        }
        
        new_heading = !new_heading;
    }
}

/************************************************************************/
/* Help function to delay sonar ping requests and new compass headings. */
/************************************************************************/
void delay_counter(int8_t state)
{
    static uint8_t delay_counter = 0;
    
    delay_counter++;
    
    if (delay_counter == 10) {
        delay_counter = 0;
        
        if (state == _DEFAULT) {
            ok_for_bucket_sensor = true;
            ok_for_top_sensor    = true;
        } else if (state == BUCKET_IN) {
            ok_for_bucket_sensor = true;
        }
        
        ok_for_compass = true;
    }
}

/************************************************************************/
/* Interrupt Service Routine for Timer4.                                */
/************************************************************************/
void timer4_isr(void)
{
    int8_t state = current_state();
    
    switch(state) {
        /// Startup states.
        /******************/
        /// Move the lifting arm to home position.
        case -4 :
            lifting_arm_down();
            break;
        /// Rotate the bucket to home position.
        case -3 :
            bucket_out();
            break;
        /// Move the catapult arm to home position.
        case -2 :
            catapult_arm_down();
            break;
        /// Move the catapult locking servo to home position.
        case -1 :
            catapult_unlock();
            break;
        
        /// Regular states.
        /******************/
        /// Default state. Wait for either sensor to give interesting input.
        case 0 :
            default_state();
            break;
        /// Rotate the bucket in hope of catching a ball.
        case 1 :
            bucket_in();
            break;
        /// In case of ball present in the bucket: Move the lifting arm to upper position.
        case 2 :
            lifting_arm_up();
            break;
        /// Move the lifting arm to lower position.
        case 3 :
            lifting_arm_down();
            break;        
        /// Rotate the bucket to home position.
        case 4 :
            bucket_out();
            break;
        /// Turn to mid wall.
        case 5 :
            turn_to_mid_wall();
            break;
        /// Turn robot in case of too close to a wall.
        case 6 :
            turn_for_wall();
            break;
        /// Turn left to prepare for launch.
        case 7 :
            turn_to_launch();
            break;
        /// Lock catapult.
        case 8 :
            catapult_lock();
            break;
        /// Tighten catapult.
        case 9 :
            catapult_arm_up();
            break;
        /// Release catapult.
        case 10 :
            catapult_unlock();
            break;
        /// Untighten catapult.
        case 11 :
            catapult_arm_down();
            break;
    }
    
    delay_counter(state);
}
