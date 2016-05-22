/************************************************************************/
/* servo.h - The .h file for servo management.                          */
/************************************************************************/

#ifndef SERVO_H
#define SERVO_H

/************************************************************************/
/* Declaration of functions used in servo.cpp                           */
/************************************************************************/
void servo_attach(uint8_t);
void servo_detach(uint8_t);
void servo_angle_decrement(uint8_t, uint8_t);
void servo_angle_increment(uint8_t, uint8_t);
bool servo_at_min_angle(uint8_t);
bool servo_at_max_angle(uint8_t);
void top_sensor_servo_rotate(void);
bool top_servo_right_angle(void);

#endif