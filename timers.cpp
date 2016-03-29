/*
 * timers.cpp
 */

#include "Arduino.h"

/* Initialization of Timer4 */
void timer4_init(void)
{
  TIMSK4 = (1 << OCIE4A);  // Interrupt enable
  TCCR4A = 0;
  TCCR4B = (1 << WGM42);  // CTC mode
  OCR4A  = 62500;  // Total timer ticks

  TCCR4B |= (1 << CS42); // 256 prescaler
}

/* Start Timer4 */
void timer4_start(void)
{
  TCCR4B |= (1 << CS42);
}

/* Stop Timer4 */
void timer4_stop(void)
{
  TCCR4B &= ~(1 << CS42);
}
