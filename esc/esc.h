#ifndef _ESCLIB_H_
#define _ESCLIB_H_


void setSpeedMotors(int motor, int speed); // set speed motor

void calibrate(void); // calibrate esc when first run

void arm(void); // set esc on arm

void stop_motor(void); // stop motors 

#endif // _ESCLIB_H