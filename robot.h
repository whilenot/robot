/************************************************************************/
/* robot.h - The main .h file for the robot project.                    */
/************************************************************************/

#ifndef ROBOT_H
#define ROBOT_H

/************************************************************************/
/* Compass definitions.                                                 */
/************************************************************************/
#define NEW_HEADING  1
#define GET_HEADING  0

/************************************************************************/
/* Sensor definitions.                                                  */
/************************************************************************/
#define MID   0
#define SIDE  1

/************************************************************************/
/* Servo definitions.                                                   */
/************************************************************************/
#define LIFTING_ARM       0
#define BUCKET_ROTATION   1
#define CATAPULT_ARM      2
#define CATAPULT_LOCKING  3
#define TOP_SENSOR        4

/************************************************************************/
/* State definitions.                                                   */
/************************************************************************/
/// Startup states.
#define LIFTING_ARM_HOME       -4
#define BUCKET_HOME            -3
#define CATAPULT_ARM_HOME      -2
#define CATAPULT_LOCKING_HOME  -1

/// Regular states.
#define _DEFAULT                0
#define BUCKET_IN               1
#define LIFTING_ARM_UP          2
#define LIFTING_ARM_DOWN        3
#define BUCKET_OUT              4
#define TURN_TO_MID_WALL        5
#define TURN_FOR_WALL           6
#define TURN_TO_LAUNCH          7
#define CATAPULT_LOCK           8
#define CATAPULT_ARM_UP         9
#define CATAPULT_UNLOCK         10
#define CATAPULT_ARM_DOWN       11

/************************************************************************/
/* Wheel definitions.                                                   */
/************************************************************************/
#define ON   1
#define OFF  0

#define FORWARD   1
#define BACKWARD  0

#define RIGHT  0
#define LEFT   1
#define BOTH   2

#endif