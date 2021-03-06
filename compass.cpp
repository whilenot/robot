/************************************************************************/
/* compass.cpp - The .cpp file for compass communication.               */
/*                                                                      */
/* This is for using the Honeywell HMC6352 compass module.              */
/************************************************************************/

#include "Arduino.h"
#include "compass.h"
#include "robot.h"
#include <util/atomic.h>
#include <Wire.h>

/// A heading between the start heading plus/minus this value is considered OK.
#define COMPASS_TRIGGER_ANGLE  4

/// 7-bit address, R/W bit controlled by Wire library.
static const uint8_t compass_address = 0x42 >> 1;

/// 8-bit address to the compass's RAM register.
static const uint8_t address_to_ram = 0x74;

/// Variable holding the start heading of the compass.
static int16_t compass_start_heading = 0;

/// Atomic variable holding the current heading of the compass.
static int16_t compass_heading_atomic = 0;

/************************************************************************/
/* Initialization of compass communication.                             */
/************************************************************************/
void compass_init(void)
{
    /// Start I2C communication.
    Wire.begin();
  
    /// Write setup to compass.
    compass_write_to_ram(0x10);
  
    /// Safety delay.
    delay(1000);
  
    /// Read and print the ram setup.
    Serial.println(compass_read_from_ram());
  
    /// Safety delay.
    delay(1000);
  
    /// Update the compass heading - step 1.
    compass_update(NEW_HEADING);
    
    /// Safety delay.
    delay(100);
    
    /// Update the compass heading - step 2.
    compass_update(GET_HEADING);
  
    /// Assign updated value to start heading variable.
    compass_start_heading = compass_heading_atomic;  
  
    Serial.print("CS: ");
    Serial.println(compass_heading_atomic);
}

/************************************************************************/
/* Calculates or retrieves new heading (@param task).                   */
/************************************************************************/
void compass_update(uint8_t task)
{
    /// Assumes Standby Mode. Performs new heading calculation.
    if (task) {
        Wire.beginTransmission(compass_address);
        Wire.write('A');  // Get heading
        Wire.endTransmission();

    /// Get heading.
    } else {
        /// Request to read two byte of data.
        Wire.requestFrom(compass_address, (uint8_t) 2);  // Casting due to compiler warning
    
        /// Read and store data (value between 0-3599).
        uint8_t MSB = Wire.read();
        uint8_t LSB = Wire.read();

        /// Calculates the heading.
        float heading = 0.1 * (( MSB << 8 ) + LSB);

        /// Possible conversion to negative value.
        if(heading > 180) heading -= 360;
    
        /// Stores the heading in an atomic variable.
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            compass_heading_atomic = (int16_t) heading;
        }
    
        Serial.print("C: ");
        Serial.println(compass_heading_atomic);        
    }
}

/************************************************************************/
/* Write desired setup (@param data) to the RAM register. See datasheet.*/
/************************************************************************/
void compass_write_to_ram(uint8_t data)
{
  /// Perform write to RAM request.
  Wire.beginTransmission(compass_address);
  Wire.write('G');  // Write to RAM register
  
  /// Necessary delay according to datasheet.
  delay(1);

  /// Continue with write to RAM operation.
  Wire.write(address_to_ram);
  
  /// Safety delay.
  delay(1);

  /// Finish write to RAM operation.
  Wire.write(data);
  Wire.endTransmission();
}

/************************************************************************/
/* @returns the current setup of the RAM register. See datasheet.       */
/************************************************************************/
uint8_t compass_read_from_ram(void)
{
    /// Perform read from RAM request.
    Wire.beginTransmission(compass_address);
    Wire.write('g');  // Read from RAM register

    /// Necessary delay according to datasheet.
    delay(1);

    /// Continue with read from RAM operation.
    Wire.write(address_to_ram);
    Wire.endTransmission();

    /// Safety delay.
    delay(1000);
  
    /// Request to read one byte of data.
    Wire.requestFrom(compass_address, (uint8_t) 1);  // Casting due to compiler warning
  
    return (uint8_t) Wire.read();
}

/************************************************************************/
/* @returns whether the heading is OK or not, when turning to mid wall. */
/************************************************************************/
bool compass_heading_ok(void)
{
    return ((compass_heading_atomic >= (compass_start_heading - COMPASS_TRIGGER_ANGLE)) && 
        (compass_heading_atomic <= (compass_start_heading + COMPASS_TRIGGER_ANGLE))) ? true : false;
}