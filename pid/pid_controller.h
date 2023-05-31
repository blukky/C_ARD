#ifndef _PIDCONTROLLERLIB_H_
#define _PIDCONTROLLERLIB_H_

int limited(int a, float min, float max); // limit data

void pid_function(float Error, float P , float I, float D, float PrevError, float PrevIterm, float min, float max, float PIDReturn[]); // function pid controller

#endif // _PIDCONTROLLERLIB_H_