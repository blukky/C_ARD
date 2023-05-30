#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <time.h>
#include <unistd.h>



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


// static int fd;


int configMPU6050(){
   int fd = wiringPiI2CSetup(MPU6050_ADDR);
   if (fd < 0){
    return -1;
   }
   wiringPiI2CWriteReg8(fd, MPU6050_SMPLRT_DIV, 0x07);
   wiringPiI2CWriteReg8(fd, MPU6050_PWR_MGMT_1, 0x01);  
   wiringPiI2CWriteReg8(fd, CONFIG, 0x06);
   wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 1<<3);
   wiringPiI2CWriteReg8(fd, ACCEL_CONFIG, 2<<3);
   wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);
   
   return fd;
}


short read_raw_data(int fd, int addr){
   short high_byte, low_byte, value;
   high_byte = wiringPiI2CReadReg8(fd, addr);
   low_byte = wiringPiI2CReadReg8(fd, addr + 1);
   value = (high_byte << 8) | low_byte;
   return value;
}

void get_accel_data(int fd, float *x, float *y, float *z){
   *x = read_raw_data(fd, ACCEL_XOUT_H) / 4096.0;
   *y = read_raw_data(fd, ACCEL_YOUT_H) / 4096.0;
   *z = read_raw_data(fd, ACCEL_ZOUT_H) / 4096.0;
}

void get_gyro_data(int fd, float *x, float *y, float *z){
   *x = read_raw_data(fd, GYRO_XOUT_H);
   *y = read_raw_data(fd, GYRO_YOUT_H);
   *z = read_raw_data(fd, GYRO_ZOUT_H);

}


float dist(float x, float y, float z){
   return sqrt(x * x + y * y + z * z);
}

void calibrateGyro(int fd, int iter, float *x_error, float *y_error, float *z_error){
   float GyroX, GyroY, GyroZ;
   for (int i=0; i<iter; i++ ){
      get_gyro_data(fd, &GyroX, &GyroY, &GyroZ);

      *x_error = *x_error + GyroX / 65.5;
      *y_error = *y_error + GyroY / 65.5;
      *z_error = *z_error + GyroZ / 65.5;
      delay(1);
   }

   *x_error = *x_error /  iter;
   *y_error = *y_error / iter;
   *z_error = *z_error /  iter;
}


float calcKalmanAngle(float acc, float gyro, float dt, float *uncertanty,  float *kalman_gain, float angle){
   angle = angle + gyro * dt;
   *uncertanty = *uncertanty + dt * dt * (*uncertanty) * (*uncertanty);
   *kalman_gain = *uncertanty * 1 / (*uncertanty + 9);
   angle += *kalman_gain * (acc - angle);
   *uncertanty = *uncertanty * (1 - *kalman_gain);
   return angle;
   }

// int main (){
   
   
//    // gyroscope params
//    int fd;
//    float AccX, AccY, AccZ;
//    float GyroX, GyroY, GyroZ;
//    float Gx, Gy, Gz;
//    float GyroPitchError = 0, AccelPitchError = 0;
//    float GyroRollError = 0, AccelRollError = 0;
//    float GyroYawError = 0;
//    float yaw = 0;
//    float Apitch = 0, Aroll = 0;

//    float kalmanPitch = 0, kalmanRoll = 0;
//    float uncertantyPitch = 4, uncertantyRoll = 4;
//    float kalmanGainPitch = 0, kalmanGainRoll = 0;

//    calibrateGyro(1000, &GyroPitchError, &GyroRollError, &GyroYawError);
   
//    get_accel_data(&AccX, &AccY, &AccZ);
//    float v = dist(AccX, AccY, AccZ);
//    if (v == 0){
//       kalmanPitch = 0;
//       kalmanRoll = 0;
//    }else{
//       kalmanPitch =  asin(AccY / v) * RAD_TO_DEG - AccelPitchError;
//       kalmanRoll =  asin(- AccX / v) * RAD_TO_DEG - AccelRollError;
//    }
//    while (1){
//       get_accel_data(&AccX, &AccY, &AccZ);
//       get_gyro_data(&GyroX, &GyroY, &GyroZ);
//       float v = dist(AccX, AccY, AccZ);
//       if (v == 0){
//          Apitch = 0;
//          Aroll = 0;
//       }else{
//          Apitch =  asin(AccY / v) * RAD_TO_DEG - AccelPitchError;
//          Aroll =  asin(- AccX / v) * RAD_TO_DEG - AccelRollError;
//       }
//       Gx = GyroX / 65.5 - GyroPitchError;
//       Gy = GyroY / 65.5 - GyroRollError;
//       Gz = GyroZ / 65.5 - GyroYawError;
//       yaw = 0.9 * yaw + 0.1 * (yaw + Gz * dt);
//       kalmanPitch = calcKalmanAngle(Apitch, Gx, dt, &uncertantyPitch, &kalmanGainPitch, kalmanPitch);
//       kalmanRoll = calcKalmanAngle(Aroll, Gy, dt, &uncertantyRoll, &kalmanGainRoll, kalmanRoll);
//       printf("pitch=%.2f roll=%.2f yaw=%.2f\n", kalmanPitch, kalmanRoll, yaw);
//       delay(4);
      
//    }
//    return 0;
// }
