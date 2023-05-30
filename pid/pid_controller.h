#ifndef _PIDCONTROLLERLIB_H_
#define _PIDCONTROLLERLIB_H_

int limited(int a, float min, float max); // limit data

float *pid_function(float k_p, float k_i, float k_d, float dt, float integral, float error, float last_error, float min, float max); // function pid controller

#endif // _PIDCONTROLLERLIB_H_