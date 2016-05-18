/************************************************************************/
/* timer.cpp - The .cpp file for using timers.                          */
/************************************************************************/

#include "Arduino.h"
#include "timer.h"

/************************************************************************/
/* Initialization of Timer4.                                            */
/************************************************************************/
void timer4_init(void)
{
  TIMSK4 = (1 << OCIE4B);  // Interrupt enable
  TCCR4A = 0;
  TCCR4B = (1 << WGM42);   // CTC mode
  OCR4A  = 624;            // Total timer ticks

  TCCR4B |= (1 << CS42);   // 256 prescaler
}

/************************************************************************/
/* Interrupt Service Routine for Timer4.                                */
/************************************************************************/
ISR(TIMER4_COMPB_vect)
{
    timer4_isr();
}