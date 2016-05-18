/************************************************************************/
/* state.h - The .h file for state management.                          */
/************************************************************************/

#ifndef STATE_H
#define STATE_H

/************************************************************************/
/* Declaration of functions used in sensor.cpp                          */
/************************************************************************/
void lifting_arm_servo_rotate(uint8_t);
void bucket_rotation_servo_rotate(uint8_t);
void catapult_arm_servo_rotate(uint8_t);
void catapult_locking_servo_rotate(uint8_t);
void default_state(void);
void turn_to_mid_wall_state(void);
void turn_for_wall_state(void);
void top_sensor_servo_rotate(void);
int8_t current_state(void); 

#endif