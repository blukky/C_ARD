#include <gyro.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <time.h>
#include <unistd.h>






void gyro_signals(){
    
}

int main()
{

    // gyroscope params
    int fd;
    float dt = 0.004;
    // float AccX, AccY, AccZ;
    // float GyroX, GyroY, GyroZ;
    // float Gx, Gy, Gz;
    // float GyroPitchError = 0, AccelPitchError = 0;
    // float GyroRollError = 0, AccelRollError = 0;
    // float GyroYawError = 0, float yaw = 0;
    // float Apitch = 0, Aroll = 0;
    // float kalmanPitch = 0, kalmanRoll = 0;
    // float uncertantyPitch = 4, uncertantyRoll = 4;
    // float kalmanGainPitch = 0, kalmanGainRoll = 0;
    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    int RateCalibrationNumber;
    uint32_t LoopTimer;
    // pid controller params
    float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
    float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
    float InputRoll, InputThrottle, InputPitch, InputYaw;
    float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
    float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
    float PIDReturn[] = {0, 0, 0};
    float PRateRoll = 0.6;
    float PRatePitch = PRateRoll;
    float PRateYaw = 2;
    float IRateRoll = 3.5;
    float IRatePitch = IRateRoll;
    float IRateYaw = 12;
    float DRateRoll = 0.03;
    float DRatePitch = DRateRoll;
    float DRateYaw = 0;
    // motors param
    float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
    // accelerometer
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch;
    // kalman param
    float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
    float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
    float Kalman1DOutput[] = {0, 0};
    float DesiredAngleRoll, DesiredAnglePitch;
    float ErrorAngleRoll, ErrorAnglePitch;
    float PrevErrorAngleRoll, PrevErrorAnglePitch;
    float PrevItermAngleRoll, PrevItermAnglePitch;
    float PAngleRoll = 2;
    float PAnglePitch = PAngleRoll;
    float IAngleRoll = 0;
    float IAnglePitch = IAngleRoll;
    float DAngleRoll = 0;
    float DAnglePitch = DAngleRoll;

    // init
    calibrateGyro(fd, 1000, &GyroPitchError, &GyroRollError, &GyroYawError);

    get_accel_data(fd, &AccX, &AccY, &AccZ);
    float v = dist(AccX, AccY, AccZ);
    if (v == 0)
    {
        kalmanPitch = 0;
        kalmanRoll = 0;
    }
    else
    {
        kalmanPitch = asin(AccY / v) * RAD_TO_DEG - AccelPitchError;
        kalmanRoll = asin(-AccX / v) * RAD_TO_DEG - AccelRollError;
    }
    while (1)
    {
        get_accel_data(&AccX, &AccY, &AccZ);
        get_gyro_data(&GyroX, &GyroY, &GyroZ);
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
        yaw = 0.9 * yaw + 0.1 * (yaw + Gz * dt);
        kalmanPitch = calcKalmanAngle(Apitch, Gx, dt, &uncertantyPitch, &kalmanGainPitch, kalmanPitch);
        kalmanRoll = calcKalmanAngle(Aroll, Gy, dt, &uncertantyRoll, &kalmanGainRoll, kalmanRoll);
        printf("pitch=%.2f roll=%.2f yaw=%.2f\n", kalmanPitch, kalmanRoll, yaw);
        delay(4);
    }
    return 0;
}