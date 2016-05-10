/*
 * timers.cpp
 */

#include "Arduino.h"

/* Initialization of Timer4 */
void timer4_init(void)
{
  TIMSK4 = (1 << OCIE4B);  // Interrupt enable
  TCCR4A = 0;
  TCCR4B = (1 << WGM42);  // CTC mode
  OCR4A  = 625;  // Total timer ticks

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

/* Initialization of Timer4 */
void timer3_init(void)
{
  TIMSK3 = (1 << OCIE3B);  // Interrupt enable
  TCCR3A = 0;
  TCCR3B = (1 << WGM32);  // CTC mode
  OCR3A  = 625;  // Total timer ticks

  TCCR3B |= (1 << CS32); // 256 prescaler
}

