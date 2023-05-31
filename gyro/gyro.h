#ifndef _GYROLIB_H_
#define _GYROLIB_H_



#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_SMPLRT_DIV 0x19
#define CONFIG 0x1a
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define INT_ENABLE 0x38
#define COMPLEMENTARY_FILTER 0.98
#define ACCEL_XOUT_H 0x3b
#define ACCEL_YOUT_H 0x3d
#define ACCEL_ZOUT_H 0x3f
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define GRAVITY_ACCEL 9.81

#define RAD_TO_DEG 180.0 / 3.14159265358979323846


int configMPU6050(void); // Init and config MPU6050

short read_raw_data(int fd, int addr); // read raw data from sensor

void get_accel_data(int fd, float *x, float *y, float *z); // get accelerometer data

void get_gyro_data(int fd, float *x, float *y, float *z); // get gyroscope data

float dist(float x, float y, float z); // calc helper params for messure angle

void calibrateGyro(int fd, int iter, float *x_error, float *y_error, float *z_error); // calibrate gyroscope

float calcKalmanAngle(float acc, float gyro, float dt, float *uncertanty,  float *kalman_gain, float angle); // calculated angle drone

void initGyro(int fd, float *kalmanPitch, float *kalmanRoll, float AccelPitchError, float AccelRollError); // init first value gyro

void get_angle(int fd, float *Gx, float *Gy, float *Gz, float *yaw, float *kalmanPitch, float *kalmanRoll,
               float AccelPitchError, float AccelRollError, float GyroPitchError, float GyroRollError, float GyroYawError,
               float *uncertantyPitch, float *uncertantyRoll, float *kalmanGainPitch, float *kalmanGainRoll); // get angle from data

#endif // _TOFLIB_H