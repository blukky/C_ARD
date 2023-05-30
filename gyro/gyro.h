#ifndef _GYROLIB_H_
#define _GYROLIB_H_


int configMPU6050(void); // Init and config MPU6050

short read_raw_data(int fd, int addr); // read raw data from sensor

void get_accel_data(int fd, float *x, float *y, float *z); // get accelerometer data

void get_gyro_data(int fd, float *x, float *y, float *z); // get gyroscope data

float dist(float x, float y, float z); // calc helper params for messure angle

void calibrateGyro(int fd, int iter, float *x_error, float *y_error, float *z_error); // calibrate gyroscope

float calcKalmanAngle(float acc, float gyro, float dt, float *uncertanty,  float *kalman_gain, float angle); // calculated angle drone

#endif // _TOFLIB_H