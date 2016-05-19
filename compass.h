/************************************************************************/
/* compass.h - The .h file for compass communication.                   */
/*                                                                      */
/* This is for using the Honeywell HMC6352 compass module.              */
/************************************************************************/

#ifndef COMPASS_H
#define COMPASS_H

/************************************************************************/
/* Declaration of functions used in compass.cpp                         */
/************************************************************************/
void compass_init(void);
void compass_update(void);
void compass_write_to_ram(uint8_t);
uint8_t compass_read_from_ram(void);
bool compass_heading_ok(void);

#endif
