/************************************************************************/
/* sensor.cpp - The .cpp file for using ultra sonic sensors.            */
/************************************************************************/

#include "Arduino.h"
#include "sensor.h"
#include <NewPing.h>
#include <util/atomic.h>

/// Arduino specific pins for ultra sonic sensors.
#define BUCKET_SENSOR_TRIG_PIN  6
#define BUCKET_SENSOR_ECHO_PIN  7
#define TOP_SENSOR_TRIG_PIN     40
#define TOP_SENSOR_ECHO_PIN     42

/// Max distances for ultra sonic sensors to report.
#define BUCKET_SENSOR_MAX_DISTANCE  20
#define TOP_SENSOR_MAX_DISTANCE     50

/// Distances for the system to get triggered.
#define BUCKET_SENSOR_TRIGGER_DISTANCE    15
#define TOP_SENSOR_TRIGGER_DISTANCE_MID   16
#define TOP_SENSOR_TRIGGER_DISTANCE_SIDE  18

/// Initialization of array holding the top sensor trigger distances.
static const uint8_t top_sensor_trigger_distance[2] = {
    TOP_SENSOR_TRIGGER_DISTANCE_MID,
    TOP_SENSOR_TRIGGER_DISTANCE_SIDE
};

/// Initialization of sonar objects.
NewPing bucket_sensor(BUCKET_SENSOR_TRIG_PIN, BUCKET_SENSOR_ECHO_PIN, BUCKET_SENSOR_MAX_DISTANCE);
NewPing top_sensor(TOP_SENSOR_TRIG_PIN, TOP_SENSOR_ECHO_PIN, TOP_SENSOR_MAX_DISTANCE);

/// Atomic variables holding the latest distance retrieved from the ultra sonic sensors.
static uint8_t bucket_sensor_distance_atomic = UINT8_MAX;
static uint8_t top_sensor_distance_atomic    = UINT8_MAX;

/************************************************************************/
/* Updates the measured distance of the bucket sensor.                  */
/************************************************************************/
void bucket_sensor_update(void)
{
    /// Retrieving distance from sensor.
    uint8_t distance = (uint8_t) bucket_sensor.ping_cm();
    
    /// Stores the distance in an atomic variable.
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        bucket_sensor_distance_atomic = distance;
    }
}

/************************************************************************/
/* Updates the measured distance of the top sensor.                     */
/************************************************************************/
void top_sensor_update(void)
{
    /// Retrieving distance from sensor.
    long time = top_sensor.ping_median(3);
    uint8_t distance = (uint8_t) top_sensor.convert_cm(time);
    
    /// Stores the distance in an atomic variable.
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        top_sensor_distance_atomic = distance;
    }
    
    Serial.print(bucket_sensor_distance_atomic);
    Serial.print("  ");
    Serial.println(top_sensor_distance_atomic);
}

/************************************************************************/
/* @returns true or false whether the bucket sensor is triggered or not.*/
/************************************************************************/
bool bucket_sensor_triggered(void)
{
    bool triggered = false;
    
    if ((bucket_sensor_distance_atomic > 0) && (bucket_sensor_distance_atomic <= BUCKET_SENSOR_TRIGGER_DISTANCE)) {
        bucket_sensor_distance_atomic = UINT8_MAX;
        triggered = true;
    }
    
    return triggered;
}

/************************************************************************/
/* @returns true or false whether the top sensor is triggered or not.   */
/************************************************************************/
bool top_sensor_triggered(uint8_t val)
{
    bool triggered = false;
    
    if ((top_sensor_distance_atomic >= 10) && (top_sensor_distance_atomic <= top_sensor_trigger_distance[val])) {
        top_sensor_distance_atomic = UINT8_MAX;
        triggered = true;
    }
    
    return triggered;
}