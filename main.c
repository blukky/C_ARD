#include <gyro.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <time.h>
#include <unistd.h>



void get_angle(int fd, float *yaw, float *kalmanPitch, float *kalmanRoll,
               float AccelPitchError, float AccelRollError, float GyroPitchError, float GyroRollError, float GyroYawError,
               float *uncertantyPitch, float *uncertantyRoll, float *kalmanGainPitch, float *kalmanGainRoll)
{
   float Apitch, Aroll;
   float Gx, Gy, Gz;
   float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
   get_accel_data(fd, &AccX, &AccY, &AccZ);
   get_gyro_data(fd, &GyroX, &GyroY, &GyroZ);
   float v = dist(AccX, AccY, AccZ);
   if (v == 0)
   {
      Apitch = 0;
      Aroll = 0;
   }
   else
   {
      Apitch = asin(AccY / v) * RAD_TO_DEG - AccelPitchError;
      Aroll = asin(-AccX / v) * RAD_TO_DEG - AccelRollError;
   }
   Gx = GyroX / 65.5 - GyroPitchError;
   Gy = GyroY / 65.5 - GyroRollError;
   Gz = GyroZ / 65.5 - GyroYawError;
   *yaw = 0.9 * *yaw + 0.1 * (*yaw + Gz * 0.004);
   *kalmanPitch = calcKalmanAngle(Apitch, Gx, 0.004, uncertantyPitch, kalmanGainPitch, *kalmanPitch);
   *kalmanRoll = calcKalmanAngle(Aroll, Gy, 0.004, uncertantyRoll, kalmanGainRoll, *kalmanRoll);
   // printf("pitch=%.2f roll=%.2f yaw=%.2f\n", *kalmanPitch, *kalmanRoll, *yaw);
}

void initGyro(int fd, float *kalmanPitch, float *kalmanRoll, float AccelPitchError, float AccelRollError)
{
   float AccX, AccY, AccZ;
   get_accel_data(fd, &AccX, &AccY, &AccZ);
   float v = dist(AccX, AccY, AccZ);
   if (v == 0)
   {
      *kalmanPitch = 0;
      *kalmanRoll = 0;
   }
   else
   {
      *kalmanPitch = asin(AccY / v) * RAD_TO_DEG - AccelPitchError;
      *kalmanRoll = asin(-AccX / v) * RAD_TO_DEG - AccelRollError;
   }
}

int main()
{

   // gyroscope params
   int fd;
   float gyroX, gyroY, gyroZ; 
   float gyroPitchError = 0, accelPitchError = 0;
   float gyroRollError = 0, accelRollError = 0;
   float gyroYawError = 0;
   float yaw = 0;
   float kalmanPitch = 0, kalmanRoll = 0;
   float uncertantyPitch = 4, uncertantyRoll = 4;
   float kalmanGainPitch = 0, kalmanGainRoll = 0;
   // init time params
   clock_t start;





   fd = configMPU6050();

   calibrateGyro(fd, 1000, &gyroPitchError, &gyroRollError, &gyroYawError);
   initGyro(fd, &kalmanPitch, &kalmanRoll, accelPitchError, accelRollError);
   printf(">[init]roll: %.2f pitch: %.2f yaw: %.2f\n", kalmanRoll, kalmanPitch, yaw);
   start = clock();
   while (1)
   {
      get_angle(fd, &yaw, &kalmanPitch, &kalmanRoll,
               accelPitchError, accelRollError, gyroPitchError, gyroRollError, gyroYawError,
               &uncertantyPitch, &uncertantyRoll, &kalmanGainPitch, &kalmanGainRoll);
      printf(">[from main] pitch=%.2f roll=%.2f yaw=%.2f\n", kalmanPitch, kalmanRoll, yaw);
      while((clock() - start) * 1000 / CLOCKS_PER_SEC < 4);
      start = clock();
   }
   return 0;
}