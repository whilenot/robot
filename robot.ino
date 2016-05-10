/*
   robot.ino
*/

#include <Servo.h>
#include <NewPing.h>
#include <util/atomic.h>

#include "encoders.h"
#include "robot.h"
#include "timers.h"
#include "wheels.h" 

/// Arduino specific pins for ultra sonic sensors 
#define BUCKET_SENSOR_TRIG_PIN  6
#define BUCKET_SENSOR_ECHO_PIN  7
#define TOP_SENSOR_TRIG_PIN     40
#define TOP_SENSOR_ECHO_PIN     42

/// Max distances for ultra sonic sensors to report
#define BUCKET_SENSOR_MAX_DISTANCE  20
#define TOP_SENSOR_MAX_DISTANCE     50

/// Distances for the system to get triggered. Top sensor is not implemented!
#define BUCKET_SENSOR_TRIGGER_DISTANCE  15
#define TOP_SENSOR_TRIGGER_DISTANCE     20

/// Arduino specific pins for servos
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

/// Delay until the lifting arm moves down (in 50 ms of a second, so 200 => 1 second delay)
#define LIFTING_ARM_DELAY_TIME      200

/// Top servo start angle value, and min- and max angle value.
#define TOP_SENSOR_SERVO_START  10
#define TOP_SENSOR_SERVO_MIN    15
#define TOP_SENSOR_SERVO_MAX    130

/// Defines to make the code more clear. These are used when calling state functions.
#define UP      0
#define DOWN    1
#define IN      0
#define OUT     1
#define LOCK    0
#define UNLOCK  1

/// Atomic variables holding the latest distance retrieved from the ultra sonic sensors.
/// Top sensor is not implemented!
static volatile uint8_t bucket_sensor_distance_atomic = 30;
static volatile uint8_t top_sensor_distance_atomic    = 200;

/// Variables holding information of the number of balls (if any) that can be launched.
static volatile bool got_second_ball    = false;
static volatile bool prepared_to_launch = false;

/// Variable holding the current state of the robot.
static volatile int state = -4;

/// Initialize sonar objects
NewPing bucket_sensor(BUCKET_SENSOR_TRIG_PIN, BUCKET_SENSOR_ECHO_PIN, BUCKET_SENSOR_MAX_DISTANCE);
NewPing top_sensor   (TOP_SENSOR_TRIG_PIN,    TOP_SENSOR_ECHO_PIN,    TOP_SENSOR_MAX_DISTANCE);

/// Servo objects
Servo lifting_arm_servo;
Servo bucket_rotation_servo;
Servo catapult_arm_servo;
Servo catapult_locking_servo;
Servo top_sensor_servo;

/// Counter variable for the top servo angle.
/// Unlike the other servo angle counters this variable is global so it can be reset in state 6.
static volatile int top_servo_angle_counter = TOP_SENSOR_SERVO_START;

/// Variable saying when it is OK for the ultra sonic sensors to do a ping request.
/// This variable is polled in the main loop.
static volatile bool ok_to_send = false;

/// Variable saying when it is OK for the top sensor servo to rotate.
/// This variable is set to true in the main loop.
static volatile bool ok_to_rotate = true;

/// Wheel speed variables
int wheelSpeedLeft  = 100;
int wheelSpeedRight = 114;


/* Initialization of the robot */
void setup()
{
  Serial.begin(9600);
  
  /// Sourcing bucket sensor with 5V
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  /* Initialize the wheels */
  wheels_init();

  /* Set the speed of the motors */
  wheels_setSpeed(LEFT,  wheelSpeedLeft);
  wheels_setSpeed(RIGHT, wheelSpeedRight);
  
  /* Two second startup delay */
  delay(2000);

  /* Initialize the encoders */
  //encoders_init();
  
  /* Toggle brakes ON or OFF */
  wheels_toggleBrake(BOTH, ON);

  timer4_init();  
}

/* Interupt Service Routine */
ISR(TIMER4_COMPB_vect)
{    
    switch(state) {
        /// Move the lifting arm to home position.
        case -4 :
            move_lifting_arm_state(DOWN);
            break;
        /// Rotate the bucket to home position.
        case -3 :
            rotate_bucket_state(OUT);
            break;
        /// Move the catapult arm to home position.
        case -2 :
            move_catapult_arm_state(DOWN);
            break;
        /// Move the catapult locking mechanism to home position.
        case -1 :
            move_catapult_locking_mechanism_state(UNLOCK);
        /// Default state. Wait for either sensor to give interesting input. Next state is either 1, 6 or 7.
        case 0 :
            default_state();
            break;
        /// Rotate the bucket in hope of catching a ball.
        /// If ball present set prepared_to_launch = true, and go to state 2. If not, go to state 4.
        case 1 :
            rotate_bucket_state(IN);
            break;
        /// In case of ball present in the bucket: Move the lifting arm to upper position.
        case 2 :
            move_lifting_arm_state(UP);
            break;
        /// Move the lifting arm to lower position.
        case 3 :
            move_lifting_arm_state(DOWN);
            break;        
        /// Rotate the bucket to home position.
        /// If got_second_ball = true, go to state 8, else go to state 0.
        case 4 :
            rotate_bucket_state(OUT);
            break;
        /// Turn against mid wall.
        case 5 :
            //turn_against_mid_wall_state();
            break;
        /// Turn robot in case of too close to a wall.
        case 6 :
            //turn_for_wall_state();
            break;
        /// Turn left to prepare for launch.
        case 7 :
            //turn_to_launch_state();
            break;
        /// Lock catapult.
        case 8 :
            move_catapult_locking_mechanism_state(LOCK);
            break;
        /// Tighten catapult.
        case 9 :
            move_catapult_arm_state(UP);
            break;
        /// Release catapult. If got_second_ball = true, go to state 2, else go to state 0.
        case 10 :
            move_catapult_locking_mechanism_state(UNLOCK);
            break;
        /// Untighten catapult.
        case 11 :
            move_catapult_arm_state(DOWN);
            break;
    }
    
    rotate_top_sensor_servo();    
    sonar_delay_counter();
}

/// Move lifting arm state (states -4, 2, 3)
void move_lifting_arm_state(byte direction) {
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

/// Rotate bucket state (states -3, 1, 4)
void rotate_bucket_state(byte direction) {
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

/// Move catapult arm state (states -2, 9, 11)
void move_catapult_arm_state(byte direction) {
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

/// Move catapult locking mechanism state (states -1, 8, 10)
void move_catapult_locking_mechanism_state(byte direction) {
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

/// Default state (state 0)
void default_state(void) {
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
            bucket_sensor_distance_atomic = 30;
            bucket_sensor_trig_counter++;
            triggered = true;
        } else {
            bucket_sensor_trig_counter = 0;
        }
    }    
    
    if (top_sensor_triggered()) {
        top_sensor_distance_atomic = 200;
        if (!prepared_to_launch) {
            if (bucket_sensor_trig_counter == 0) {
                /// Change to make turn state (state 6)
                //wheels_toggleBrake(BOTH, ON);
                
                if (top_servo_angle_counter <= 70) {
                    /// Turn left
                    wheels_setDirection(LEFT, BACKWARD);
                } else {
                    /// Turn right
                    wheels_setDirection(RIGHT, BACKWARD);
                }
                
                state = 6;
            }
        } else {
            prepared_to_launch = false;
            /// Turn left to prepare for launch (state 7)
            state = 8;
        }
    }
}

/// Turn for wall state (6)
void turn_for_wall_state(void) {
    static int delay_counter = 0;
    
    delay_counter++;
    
    if (delay_counter == 200) {
        delay_counter = 0;
        
        wheels_setDirection(LEFT, FORWARD);
        wheels_setDirection(RIGHT, FORWARD);
        
        state = 0;
    }    
}

/// Returns true or false whether the bucket sensor is triggered or not
bool bucket_sensor_triggered(void) {
    return ((bucket_sensor_distance_atomic > 0) && (bucket_sensor_distance_atomic <= BUCKET_SENSOR_TRIGGER_DISTANCE)) ? true : false;
}

/// Returns true or false whether the top sensor is triggered or not
bool top_sensor_triggered(void) {
    return ((top_sensor_distance_atomic > 0) && (top_sensor_distance_atomic <= TOP_SENSOR_TRIGGER_DISTANCE)) ? true : false;
}

/// Rotation of the top sensor servo
void rotate_top_sensor_servo(void) {    
    static bool rotate_left = true;
    
    if (ok_to_rotate) {
        ok_to_rotate = false;    
        
        if (rotate_left) {
            top_servo_angle_counter+=5;
        
            if (top_servo_angle_counter == TOP_SENSOR_SERVO_MAX) {
                rotate_left = false;
            }
        } else {
            top_servo_angle_counter-=5;
        
            if (top_servo_angle_counter == TOP_SENSOR_SERVO_MIN) {
                rotate_left = true;
            }
        }
        
        top_sensor_servo.attach(TOP_SENSOR_SERVO_PIN);
        top_sensor_servo.write(top_servo_angle_counter);
        
    } else {
        top_sensor_servo.detach();
    }
}

/// Counter to delay sonar ping requests
void sonar_delay_counter(void) {
  static int delay_counter = 0;
  
  delay_counter++;
  
  if (delay_counter == 10) {
    if (state == 0 || state == 1) {
      ok_to_send = true;
    }
    
    delay_counter = 0;
  }
}

/* Main loop */
void loop()
{
    while(!ok_to_send);

    ok_to_send = false;
  
    //uint8_t bucket_distance = bucket_sensor.ping_cm();
    uint8_t top_distance    = top_sensor.ping_cm();
  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        //bucket_sensor_distance_atomic = bucket_distance;
        top_sensor_distance_atomic    = top_distance;
    }
    
    //Serial.print(bucket_sensor_distance_atomic);
    //Serial.print("  ");
    Serial.println(top_sensor_distance_atomic);
    
    ok_to_rotate = true;
}
