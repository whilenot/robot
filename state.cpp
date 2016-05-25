/************************************************************************/
/* state.cpp - The .cpp file for state management.                      */
/************************************************************************/

#include "Arduino.h"
#include "state.h"
#include "compass.h"
#include "robot.h"
#include "sensor.h"
#include "servo.h"
#include "wheel.h"

/// Time before a ball triggers the system (in 10 ms resolution, so 100 => 1 second delay).
#define BUCKET_SENSOR_TRIG_TIME  30

/// Delay until the lifting arm moves down (10 ms resolution).
#define LIFTING_ARM_DELAY_TIME  100

/// Delay time for the robot to go back before it makes a turn (10 ms resolution).
#define GO_BACK_DELAY_TIME  75

/// Delay time for the robot to make a turn (10 ms resolution).
#define MAKE_TURN_DELAY_TIME  125

/// Time before a compass heading search is considered timed out (10 ms resolution).
#define COMPASS_TIME_OUT  1000

/// Time to delay next compass heading search (10 ms resolution).
/// This delay might be delayed itself if the robot picks up a ball or has to turn for wall.
#define TURN_TO_MID_WALL_DELAY  600

/// Maximum value of bucket trigger signals before the robot desides to turn for wall.
#define BACK_AWAY_TRIG_COUNT  3

/// Variable holding the current state of the robot.
static volatile int8_t state = INT8_MAX;

/// Variables holding information of the number of balls (if any) that can be launched.
static volatile bool got_second_ball    = false;
static volatile bool prepared_to_launch = false;

/// Variable saying what way to turn.
static volatile bool turn_left = false;

/// Variable saying if the robot is trying to find the mid wall.
static volatile bool searching_for_mid_wall = false;

/// Pre-declaration of help function.
void next_state(int8_t);

/************************************************************************/
/* Initialization of state management.                                  */
/************************************************************************/
void state_init(void)
{
    /// Setting first state.
    next_state(LIFTING_ARM_HOME);
}

/************************************************************************/
/* Default state (state 0).                                             */
/************************************************************************/
void default_state(void) 
{
    /// Counter variables for trigger check.
    static uint8_t bucket_sensor_trig_counter = 0;
    static uint8_t delay_counter = 0;
    
    /// Logic variable for trigger check.
    static bool triggered = false;
    
    /// Logic variables to ensure that the most important state is up next.
    bool pick_up_ball  = false;
    bool turn_for_wall = false;
    
    /// Counter variable for search for mid wall delay.
    static uint16_t mid_wall_counter = 0;
    
    /// Variable for counting bucket trigger signals when close to a wall.
    /// This variable helps delay the time to turn for wall in case of ball in sight.
    static uint8_t back_away_counter = 0;
    
    /// Variable holding the direction of the top sensor servo.
    uint8_t val = MID;
    
    /// The robot has less than two balls.
    if (!got_second_ball) {
        /// Bucket trigger check to make sure there is ball to pick up.
        /**************************************************************/
        
        /// Trigger delay.
        if (triggered) {
            delay_counter++;
        
            if (delay_counter == (BUCKET_SENSOR_TRIG_TIME / 2)) {
                delay_counter = 0;
                triggered = false;
            }
            
        /// Checking bucket sensor.
        } else if (bucket_sensor_triggered()) {
            bucket_sensor_trig_counter++;
            triggered = true;
            
        /// Resetting counter.
        } else {
            bucket_sensor_trig_counter = 0;
            
            wheel_toggle_brake(BOTH, OFF);
        }
        
        /// A ball will be picked up.
        if (bucket_sensor_trig_counter == 3) {
            bucket_sensor_trig_counter = 0;
            
            /// Setting logic variables.
            triggered = false;
            pick_up_ball = true;
            
            /// Stop the robot.            
            wheel_toggle_brake(BOTH, ON);
            
            next_state(BUCKET_IN);
        }
    }
    
    /// Checking the direction of the top sensor servo.
    if (!top_servo_at_mid()) {
        val = SIDE;
    }
    
    /// Checking if the robot is too close to a wall.
    /************************************************/
    if (top_sensor_triggered(val) && !pick_up_ball) {
        /// The robot has one or two balls.
        if (prepared_to_launch && !searching_for_mid_wall) {
            prepared_to_launch = false;
            
            /// Low Speed Mode for turning.
            wheel_set_speed(LOW);
            
            /// Prepare to move backwards.
            wheel_set_direction(BOTH, BACKWARD);
            
            next_state(TURN_TO_LAUNCH);
            
        /// The robot has no ball.
        } else {
            /// No ball in front of the robot.
            if ((bucket_sensor_trig_counter == 0) || (back_away_counter == BACK_AWAY_TRIG_COUNT)) {
                /// Resets the counter for trigger signals close to a wall.
                back_away_counter = 0;
                
                /// Sets logic variable to prevent another change of state.
                turn_for_wall = true;
                
                /// Low Speed Mode for turning.
                wheel_set_speed(LOW);
                
                /// Wall on the right.
                if (top_servo_right_angle()) {
                    /// Turn left.
                    turn_left = true;
                }
                
                /// Prepare to move backwards.
                wheel_set_direction(BOTH, BACKWARD);
                
                /// Let go of the robot.
                wheel_toggle_brake(BOTH, OFF);
                
                next_state(TURN_FOR_WALL);
                
            /// There might be a ball to pick up.
            } else {
                /// Increments the counter for trigger signals close to a wall.
                back_away_counter++;
                
                /// Stop the robot.
                wheel_toggle_brake(BOTH, ON);
                
            }
        }
    }
    
    /// Waiting to try to find the mid wall once again.
    /**************************************************/
    if (searching_for_mid_wall && !pick_up_ball && !turn_for_wall) {        
        mid_wall_counter++;
        
        if (mid_wall_counter == TURN_TO_MID_WALL_DELAY) {
            mid_wall_counter = 0;
            searching_for_mid_wall = false;
            
            /// Low Speed Mode for turning.
            wheel_set_speed(LOW);
                
            /// Turn left.
            wheel_set_direction(LEFT, BACKWARD);
                
            /// Let go of the robot.
            wheel_toggle_brake(BOTH, OFF);
            
            next_state(TURN_TO_MID_WALL);
        }
    }
}

/************************************************************************/
/* Rotate the bucket rotation servo IN (states 1).                      */
/************************************************************************/
void bucket_in(void)
{
    /// Increments servo angle.
    servo_angle_increment(BUCKET_ROTATION, 4);
    
    /// The servo is at max angle.
    if (servo_at_max_angle(BUCKET_ROTATION)) {
        /// Double-check if there is a ball.
        if (bucket_sensor_triggered()) {
            /// The robot has no ball from before.
            if (!prepared_to_launch) {
                prepared_to_launch = true;
                
                next_state(LIFTING_ARM_UP);
            
            /// The robot has one ball from before.
            } else {
                got_second_ball = true;
                
                /// Let go of the robot.
                wheel_toggle_brake(BOTH, OFF);
                
                next_state(_DEFAULT);
            }
        /// There is no ball. 
        } else {
            next_state(BUCKET_OUT);
        }
    }
}

/************************************************************************/
/* Rotate the bucket rotation servo OUT (states -3, 4).                 */
/************************************************************************/
void bucket_out(void)
{
    /// Decrements servo angle.
    servo_angle_decrement(BUCKET_ROTATION, 2);
    
    /// The servo is at min angle.
    if (servo_at_min_angle(BUCKET_ROTATION)) {
        /// Startup state.
        if (state == BUCKET_HOME) {
            next_state(CATAPULT_ARM_HOME);
            
        /// Regular state.
        } else if (state == BUCKET_OUT) {
            /// The robot had two balls and will launch again.
            if (got_second_ball) {
                got_second_ball = false;
                
                next_state(CATAPULT_LOCK);
                
            /// One ball is picked up - look for mid wall.
            } else if (prepared_to_launch) {
                /// Low Speed Mode for turning.
                wheel_set_speed(LOW);
                
                /// Prepare the robot to move backwards.
                wheel_set_direction(BOTH, BACKWARD);
                
                /// Let go of the robot.
                wheel_toggle_brake(BOTH, OFF);
                
                next_state(TURN_TO_MID_WALL);
                
            /// There is no ball.
            } else {
                /// Let go of the robot.
                wheel_toggle_brake(BOTH, OFF);
                
                next_state(_DEFAULT);
            }
        }
    }
}

/************************************************************************/
/* Move the lifting arm UP (state 2).                                   */
/************************************************************************/
void lifting_arm_up(void)
{
    /// Delay counter variable.
    static uint8_t delay_counter = 0;    
    
    /// The servo is not at max angle.
    if (!servo_at_max_angle(LIFTING_ARM)) {
        /// Increments servo angle.
        servo_angle_increment(LIFTING_ARM, 2);
        
    /// The servo is at max angle - delay the down movement.
    } else {
        delay_counter++;
        
        if (delay_counter == LIFTING_ARM_DELAY_TIME) {
            delay_counter = 0;
            
            next_state(LIFTING_ARM_DOWN);
        }
    }
}

/************************************************************************/
/* Move the lifting arm DOWN (states -4, 3).                            */
/************************************************************************/
void lifting_arm_down(void)
{
    /// Decrements servo angle.
    servo_angle_decrement(LIFTING_ARM, 1);
    
    /// The servo is at min angle.
    if (servo_at_min_angle(LIFTING_ARM)) {
        /// Startup state.
        if (state == LIFTING_ARM_HOME) {
            next_state(BUCKET_HOME);
            
        /// Regular state.
        } else if (state == LIFTING_ARM_DOWN) {
            next_state(BUCKET_OUT);
        }
    }
}

/************************************************************************/
/* Turn to mid wall (state 5).                                          */
/************************************************************************/
void turn_to_mid_wall(void)
{
    /// Delay counter for moving backwards.
    static uint16_t go_back_counter = 0;
    
    /// Variable saying if the robot should move backwards or not.
    static bool go_back = true;
    
    /// Time out counter.
    static uint16_t time_out = 0;
    
    /// Variable saying if the robot should move on to next state or not.
    bool move_on = false;
    
    /// Move backwards.
    if (go_back) {
        go_back_counter++;
        
        if (go_back_counter == GO_BACK_DELAY_TIME) {
            go_back_counter = 0;
            go_back = false;
            
            /// Turn left.
            wheel_set_direction(RIGHT, FORWARD);
        }
        
    /// Turn around.
    } else {
        time_out++;
    
        /// Compass heading is OK.
        if (compass_heading_ok()) {
            move_on = true;        
        
        /// Searching for compass heading timed out.
        } else if (time_out == COMPASS_TIME_OUT) {
            searching_for_mid_wall = true;
            move_on = true;
        }
        
        /// Move on to next state.
        if (move_on) {
            time_out = 0;
            go_back = true;
        
            /// Stop turning.
            wheel_set_direction(BOTH, FORWARD);
        
            /// High Speed Mode.
            wheel_set_speed(HIGH);
        
            next_state(_DEFAULT);
        }
    }    
}

/************************************************************************/
/* Turn for wall (state 6).                                             */
/************************************************************************/
void turn_for_wall(void)
{
    /// Delay counter variables.
    static uint8_t delay_counter = 0;
    static uint16_t go_back_counter = 0;
    
    /// Variable saying if the robot should move backwards or not.
    static bool go_back = true;    
    
    /// Move backwards.
    if (go_back) {
        go_back_counter++;
        
        if (go_back_counter == GO_BACK_DELAY_TIME) {
            go_back_counter = 0;
            go_back = false;
            
            /// Turn left.
            if (turn_left) {
                turn_left = false;
                
                wheel_set_direction(RIGHT, FORWARD);
                
            /// Turn right.
            } else {
                wheel_set_direction(LEFT, FORWARD);
                
            }
        }
        
    /// Turn around.
    } else {
        delay_counter++;
        
        /// Turn is done.
        if (delay_counter == MAKE_TURN_DELAY_TIME) {
            delay_counter = 0;
            go_back = true;
        
            /// Stop turning.
            wheel_set_direction(BOTH, FORWARD); 

            /// High Speed Mode.
            wheel_set_speed(HIGH);
        
            next_state(_DEFAULT);
        }
    }    
}

/************************************************************************/
/* Turn to launch (state 7).                                            */
/************************************************************************/
void turn_to_launch(void)
{
    /// Delay counter variables.
    static uint8_t delay_counter = 0;
    static uint16_t go_back_counter = 0;
    
    /// Variable saying if the robot should move backwards or not.
    static bool go_back = true;
    
    /// Move backwards.
    if (go_back) {
        go_back_counter++;
        
        if (go_back_counter == GO_BACK_DELAY_TIME) {
            go_back_counter = 0;
            go_back = false;
            
            /// Turn left.
            wheel_set_direction(RIGHT, FORWARD);
        }
    
    /// Turn around.
    } else {
        delay_counter++;
    
        /// Turn is done.
        if (delay_counter == MAKE_TURN_DELAY_TIME) {
            delay_counter = 0;
            go_back = true;
        
            /// Stop turning.
            wheel_set_direction(BOTH, FORWARD); 

            /// High Speed Mode.
            wheel_set_speed(HIGH);
        
            /// Stop the robot.
            wheel_toggle_brake(BOTH, ON);
        
            next_state(CATAPULT_LOCK);
        }    
    }    
}

/************************************************************************/
/* Lock the catapult (state 8).                                         */
/************************************************************************/
void catapult_lock(void)
{
    /// Increments servo angle.
    servo_angle_increment(CATAPULT_LOCKING, 3);
    
    /// The servo is at max angle.
    if (servo_at_max_angle(CATAPULT_LOCKING)) {
        next_state(CATAPULT_ARM_UP);
    }
}

/************************************************************************/
/* Unlock the catapult (states -1, 10).                                 */
/************************************************************************/
void catapult_unlock(void)
{
    /// Decrements servo angle.
    servo_angle_decrement(CATAPULT_LOCKING, 3);
    
    /// The servo is at min angle.
    if (servo_at_min_angle(CATAPULT_LOCKING)) {
        /// Startup state.
        if (state == CATAPULT_LOCKING_HOME) {
            next_state(_DEFAULT);
            
        /// Regular state.
        } else if (state == CATAPULT_UNLOCK) {
            next_state(CATAPULT_ARM_DOWN);
        }
    }
}

/************************************************************************/
/* Move the catapult arm UP (state 9).                                  */
/************************************************************************/
void catapult_arm_up(void)
{
    /// Increments servo angle.
    servo_angle_increment(CATAPULT_ARM, 1);
    
    /// The servo is at max angle.
    if (servo_at_max_angle(CATAPULT_ARM)) {
        next_state(CATAPULT_UNLOCK);
    }
}

/************************************************************************/
/* Move the catapult arm DOWN (states -2, 11).                          */
/************************************************************************/
void catapult_arm_down(void)
{
    /// Decrements servo angle.
    servo_angle_decrement(CATAPULT_ARM, 1);
    
    /// The servo is at min angle.
    if (servo_at_min_angle(CATAPULT_ARM)) {
        /// Startup state.
        if (state == CATAPULT_ARM_HOME) {
            next_state(CATAPULT_LOCKING_HOME);
            
        /// Regular state.
        } else if (state == CATAPULT_ARM_DOWN) {
            /// Launching is done.
            if(!got_second_ball) {
                /// Let go of the robot.
                wheel_toggle_brake(BOTH, OFF);
                
                next_state(_DEFAULT);
                
            /// There is a second ball to launch.
            } else {
                next_state(LIFTING_ARM_UP);
            }
        }
    }
}

/************************************************************************/
/* @returns if the robot is going for mid wall or not.                  */
/************************************************************************/
bool going_for_mid_wall(void)
{
    bool mid_wall = false;
    
    if (prepared_to_launch && !searching_for_mid_wall) {
        mid_wall = true;
    }
    
    return mid_wall;
}

/************************************************************************/
/* Help function to attach/detach servos and setting the next state.    */
/************************************************************************/
void next_state(int8_t next_state)
{
    /// Current state.
    switch (state) {
        /// Startup states.
        /*******************/
        case LIFTING_ARM_HOME :       // (-4)
            servo_detach(LIFTING_ARM);
            break;
        case BUCKET_HOME :            // (-3)
            servo_detach(BUCKET_ROTATION);
            break;
        case CATAPULT_ARM_HOME :      // (-2)
            servo_detach(CATAPULT_ARM);
            break;
        case CATAPULT_LOCKING_HOME :  // (-1)
            servo_detach(CATAPULT_LOCKING);
            break;
            
        /// Regular states.
        /*******************/
        case _DEFAULT :               // (0)
            servo_detach(TOP_SENSOR);
            servo_detach(LIFTING_ARM);
            break;
        case BUCKET_IN :              // (1)
            servo_detach(BUCKET_ROTATION);
            break;
        case LIFTING_ARM_UP :         // (2)
            //servo_detach(LIFTING_ARM);
            break;
        case LIFTING_ARM_DOWN :       // (3)
            servo_detach(LIFTING_ARM);
            break;
        case BUCKET_OUT :             // (4)
            servo_detach(BUCKET_ROTATION);
            servo_detach(LIFTING_ARM);
            break;
        case CATAPULT_LOCK :          // (8)
            servo_detach(CATAPULT_LOCKING);
            break;
        case CATAPULT_ARM_UP :        // (9)
            //servo_detach(CATAPULT_ARM);
            break;
        case CATAPULT_UNLOCK :        // (10)
            servo_detach(CATAPULT_LOCKING);
            break;
        case CATAPULT_ARM_DOWN :      // (11)
            servo_detach(CATAPULT_ARM);
            break;
    }
    
    /// Next state.
    switch (next_state) {
        /// Startup states.
        /*******************/
        case LIFTING_ARM_HOME :       // (-4)
            servo_attach(LIFTING_ARM);
            break;
        case BUCKET_HOME :            // (-3)
            servo_attach(BUCKET_ROTATION);
            break;
        case CATAPULT_ARM_HOME :      // (-2)
            servo_attach(CATAPULT_ARM);
            break;
        case CATAPULT_LOCKING_HOME :  // (-1)
            servo_attach(CATAPULT_LOCKING);
            break;
        
        /// Regular states.
        /*******************/
        case _DEFAULT :               // (0)
            servo_attach(TOP_SENSOR);
            servo_attach(LIFTING_ARM);
            break;
        case BUCKET_IN :              // (1)
            servo_attach(BUCKET_ROTATION);
            servo_attach(LIFTING_ARM);
            break;
        case LIFTING_ARM_UP :         // (2)
            servo_attach(LIFTING_ARM);
            break;
        case LIFTING_ARM_DOWN :       // (3)
            //servo_attach(LIFTING_ARM);
            break;
        case BUCKET_OUT :             // (4)
            servo_attach(BUCKET_ROTATION);
            break;
        case CATAPULT_LOCK :          // (8)
            servo_attach(CATAPULT_LOCKING);
            break;
        case CATAPULT_ARM_UP :        // (9)
            servo_attach(CATAPULT_ARM);
            break;
        case CATAPULT_UNLOCK :        // (10)
            servo_attach(CATAPULT_LOCKING);
            break;
        case CATAPULT_ARM_DOWN :      // (11)
            //servo_attach(CATAPULT_ARM);
            break;
    }
    
    state = next_state;
}

/************************************************************************/
/* @returns the current state of the system.                            */
/************************************************************************/
int8_t current_state(void)
{
    return state;
}