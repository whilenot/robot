/************************************************************************/
/* sensor.h - The .h file for using ultra sonic sensors.                */
/************************************************************************/

#ifndef SENSOR_H
#define SENSOR_H

/************************************************************************/
/* Declaration of functions used in sensor.cpp (needed elsewhere).      */
/************************************************************************/
void bucket_sensor_update(void);
void top_sensor_update(void);
bool bucket_sensor_triggered(void);
bool top_sensor_triggered(void);

#endif