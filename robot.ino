/************************************************************************/
/* robot.ino - The main file for the robot project.                     */
/************************************************************************/

#include "robot.h"
#include "compass.h"
#include "sensor.h"
#include "state.h"
#include "timer.h"
#include "wheel.h"

/// Variable saying when it is OK for the ultra sonic sensors to do a ping request.
/// This variable is polled in the main loop.
static volatile bool ok_to_send = false;

/// Variable saying when it is OK to update the compass heading.
/// This variable is polled in the main loop.
static volatile bool ok_for_compass = false;

/************************************************************************/
/* Initialization of the robot.                                         */
/************************************************************************/
void setup()
{
    /// Start serial communication.
    Serial.begin(9600);

    /// Initialization of the wheels.
    wheel_init();
    
    /// Initialization of the compass.
    compass_init();
  
    /// Two second startup delay.
    delay(2000);
  
    /// Toggle brakes ON or OFF.
    wheel_toggle_brake(BOTH, ON);

    /// Initialization of Timer4.
    timer4_init();
}

/************************************************************************/
/* Program main loop.                                                   */
/************************************************************************/
void loop()
{
    if (ok_to_send) {
        ok_to_send = false;
        
        bucket_sensor_update();
        top_sensor_update();
        
        top_sensor_servo_rotate();
    }
    
    if (ok_for_compass) {
        ok_for_compass = false;
        
        compass_update();
    }
}

/************************************************************************/
/* Help function to delay sonar ping requests and new compass headings. */
/************************************************************************/
void delay_counter(int8_t state)
{
    static uint8_t compass_counter = 0;
    static uint8_t sonar_counter   = 0;
    
    compass_counter++;
    sonar_counter++;
    
    if (compass_counter == 50) {
        ok_for_compass = true;
        
        compass_counter = 0;
    }
  
    if (sonar_counter == 10) {
        if (state == 0 || state == 1) {
            ok_to_send = true;
        }
    
        sonar_counter = 0;
    }
}

/************************************************************************/
/* Interrupt Service Routine for Timer4.                                */
/************************************************************************/
void timer4_isr(void)
{
    int8_t state = current_state();
    
    switch(state) {
        /// Move the lifting arm to home position.
        case -4 :
            lifting_arm_servo_rotate(DOWN);
            break;
        /// Rotate the bucket to home position.
        case -3 :
            bucket_rotation_servo_rotate(OUT);
            break;
        /// Move the catapult arm to home position.
        case -2 :
            catapult_arm_servo_rotate(DOWN);
            break;
        /// Move the catapult locking mechanism to home position.
        case -1 :
            catapult_locking_servo_rotate(UNLOCK);
        /// Default state. Wait for either sensor to give interesting input. Next state is either 1, 6 or 7.
        case 0 :
            default_state();
            break;
        /// Rotate the bucket in hope of catching a ball.
        /// If ball present set prepared_to_launch = true, and go to state 2. If not, go to state 4.
        case 1 :
            bucket_rotation_servo_rotate(IN);
            break;
        /// In case of ball present in the bucket: Move the lifting arm to upper position.
        case 2 :
            lifting_arm_servo_rotate(UP);
            break;
        /// Move the lifting arm to lower position.
        case 3 :
            lifting_arm_servo_rotate(DOWN);
            break;        
        /// Rotate the bucket to home position.
        /// If got_second_ball = true, go to state 8, else go to state 0.
        case 4 :
            bucket_rotation_servo_rotate(OUT);
            break;
        /// Turn to mid wall.
        case 5 :
            turn_to_mid_wall_state();
            break;
        /// Turn robot in case of too close to a wall.
        case 6 :
            turn_for_wall_state();
            break;
        /// Turn left to prepare for launch.
        case 7 :
            //turn_to_launch_state();
            break;
        /// Lock catapult.
        case 8 :
            catapult_locking_servo_rotate(LOCK);
            break;
        /// Tighten catapult.
        case 9 :
            catapult_arm_servo_rotate(UP);
            break;
        /// Release catapult. If got_second_ball = true, go to state 2, else go to state 0.
        case 10 :
            catapult_locking_servo_rotate(UNLOCK);
            break;
        /// Untighten catapult.
        case 11 :
            catapult_arm_servo_rotate(DOWN);
            break;
    }
    
    delay_counter(state);
}
