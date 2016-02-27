/*
 * wheels.h
 */

#ifndef _WHEELS_H_
#define _WHEELS_H_

#ifdef __cplusplus
extern "C" {
#endif

void initWheels();
void brake(char, int);
void dualBreak(int);
void dir(char, int);
void speed(char, int);

#ifdef __cplusplus
}
#endif

#endif
