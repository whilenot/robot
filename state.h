/************************************************************************/
/* state.h - The .h file for state management.                          */
/************************************************************************/

#ifndef STATE_H
#define STATE_H

/************************************************************************/
/* Declaration of functions used in sensor.cpp (needed elsewhere).      */
/************************************************************************/
void state_init(void);
void compass_calibration_state(void);
void default_state(void);
void bucket_in(void);
void bucket_out(void);
void lifting_arm_up(void);
void lifting_arm_down(void);
void turn_to_mid_wall(void);
void turn_for_wall(void);
void turn_to_launch(void);
void catapult_arm_up(void);
void catapult_arm_down(void);
void catapult_lock(void);
void catapult_unlock(void);
int8_t current_state(void);

#endif