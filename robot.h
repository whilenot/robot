/************************************************************************/
/* robot.h - The main .h file for the robot project.                    */
/************************************************************************/

#ifndef ROBOT_H
#define ROBOT_H

/************************************************************************/
/* Logic definitions.                                                   */
/************************************************************************/
/// TRUE
#define ON        1
#define UP        1
#define IN        1
#define LOCK      1
#define FORWARD   1

/// FALSE
#define OFF       0
#define DOWN      0
#define OUT       0
#define UNLOCK    0
#define BACKWARD  0

/************************************************************************/
/* State definitions.                                                   */
/************************************************************************/
#define _DEFAULT           0
#define BUCKET_IN          1
#define LIFTING_ARM_UP     2
#define LIFTING_ARM_DOWN   3
#define BUCKET_OUT         4
#define TURN_TO_MID_WALL   5
#define TURN_FOR_WALL      6
#define TURN_TO_LAUNCH     7
#define CATAPULT_LOCK      8
#define CATAPULT_ARM_UP    9
#define CATAPULT_UNLOCK    10
#define CATAPULT_ARM_DOWN  11

/************************************************************************/
/* Wheel definitions.                                                   */
/************************************************************************/
#define RIGHT     0
#define LEFT      1
#define BOTH      2

#endif
