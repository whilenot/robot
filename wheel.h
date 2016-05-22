/************************************************************************/
/* wheel.h - The .h file for controlling the wheels of the robot.       */
/************************************************************************/

#ifndef WHEEL_H
#define WHEEL_H

/************************************************************************/
/* Declaration of functions used in wheel.cpp (needed elsewhere).       */
/************************************************************************/
void wheel_init(void);
void wheel_toggle_brake(uint8_t, uint8_t);
void wheel_set_direction(uint8_t, uint8_t);
void wheel_set_speed(uint8_t);

#endif