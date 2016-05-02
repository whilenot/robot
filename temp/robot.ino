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

/// Arduino specific pins for servos
#define LIFTING_ARM_SERVO_PIN       5
#define BUCKET_ROTATION_SERVO_PIN   10
#define CATAPULT_ARM_SERVO_PIN      2
#define CATAPULT_LOCKING_SERVO_PIN  4

/// Distances for the system to get triggered. Top sensor is not implemented!
#define BUCKET_SENSOR_TRIGGER_DISTANCE  15
#define TOP_SENSOR_TRIGGER_DISTANCE     30

/// Servo max angle values. Be careful here!
#define LIFTING_ARM_SERVO_MAX       60
#define BUCKET_ROTATION_SERVO_MAX   200
#define CATAPULT_ARM_SERVO_MAX      120
#define CATAPULT_LOCKING_SERVO_MAX  60

/// Delay until the lifting arm moves down (in 50 ms of a second, so 200 => 1 second delay)
#define LIFTING_ARM_DELAY_TIME      200

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

static volatile int state = -4;

/// Sonar pin constants
static const int Trig = 6;
static const int Echo = 7;

/// Sonar max distance
static const int MaxDistance = 30;

/// Initialize sonar object
NewPing sonar(Trig, Echo, MaxDistance);

/// Servo objects
Servo lifting_arm_servo;
Servo bucket_rotation_servo;
Servo catapult_arm_servo;
Servo catapult_locking_servo;

/// Variable saying when it is OK for the ultra sonic sensor to do a ping request.
/// This variable is polled in the main loop.
static volatile bool OKtoSend = false;

/// Wheel speed variables
int wheelSpeedLeft  = 100;
int wheelSpeedRight = 114;


/* Initialization of the robot */
void setup()
{
  Serial.begin(9600);

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
        /// Turn against north.
        case 5 :
            break;
        /// Turn robot in case of too close to a wall.
        case 6 :
            break;
        /// Turn left to prepare for launch.
        case 7 :
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
            if (!prepared_to_launch) {
                state = 0;
            } else {
                state = 8;
            }            
        } else {
            state++;
        }        
    } else if (servo_angle_counter == BUCKET_ROTATION_SERVO_MAX) {
        bucket_rotation_servo.detach();
        
        if (bucket_sensor_triggered()) {
            prepared_to_launch = true;
            state++;
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
            prepared_to_launch = false;
            state = 0;
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

/// Returns true or false whether the bucket sensor is triggered or not
bool bucket_sensor_triggered(void) {
    return (bucket_sensor_distance_atomic <= BUCKET_SENSOR_TRIGGER_DISTANCE) ? true : false;
}

/// Counter to delay sonar ping requests
void sonar_delay_counter(void) {
  static int delay_counter = 0;
  
  delay_counter++;
  
  if (delay_counter == 10) {
    if (state == 0 || state == 1) {
      OKtoSend = true;
    }
    
    delay_counter = 0;
  }
}

/* Main loop */
void loop()
{
    while(!OKtoSend);

    OKtoSend = false;
  
    uint8_t distance = sonar.ping_cm();
  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bucket_sensor_distance_atomic = distance;
    }
    
    Serial.println(bucket_sensor_distance_atomic);
}
