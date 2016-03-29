/*
 * wheels.h
 */

#ifndef WHEELS_H
#define WHEELS_H

void wheels_init         (void);
void wheels_toggleBrake  (char ch, int val);
void wheels_setDirection (char ch, int val);
void wheels_setSpeed     (char ch, int val);
int  wheels_getCurrent   (char ch);

#endif
