/************************************************************************/
/* compass.h - The .h file for compass communication.                   */
/*                                                                      */
/* This is for using the Honeywell HMC6352 compass module.              */
/************************************************************************/

#ifndef COMPASS_H
#define COMPASS_H

/************************************************************************/
/* Declaration of functions used in compass.cpp (needed elsewhere).     */
/************************************************************************/
void compass_init(void);
void compass_update(uint8_t);
void compass_write_to_ram(uint8_t);
uint8_t compass_read_from_ram(void);
bool compass_heading_ok(void);

#endif