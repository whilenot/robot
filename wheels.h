/*
 * wheels.h
 */

#ifndef WHEELS_H
#define WHEELS_H

class Wheels
{
  /* Constructor */
  public:
    void toggleBrake  (char ch, int val);
    void setDirection (char ch, int val);
    void setSpeed     (char ch, int val);
    Wheels();
};

#endif
