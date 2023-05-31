#include <gyro.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <pid_controller.h>

int main()
{

    // desired params
    int InputThrottle = 1200;
    int ThrottleMin = 1180;
    float desiredRoll = 0;
    float desiredPitch = 0;
    float desiredYaw = 0;

    // pid controller params
    float PRateRoll = 0.6;
    float PRatePitch = PRateRoll;
    float PRateYaw = 2;
    float IRateRoll = 3.5;
    float IRatePitch = IRateRoll;
    float IRateYaw = 12;
    float DRateRoll = 0.03;
    float DRatePitch = DRateRoll;
    float DRateYaw = 0;
    float PAngleRoll = 2;
    float PAnglePitch = PAngleRoll;
    float IAngleRoll = 0;
    float IAnglePitch = IAngleRoll;
    float DAngleRoll = 0;
    float DAnglePitch = DAngleRoll;

    float PIDResult[3];

    // pitch
    float errorPitch = 0;
    float prevErrorPitch = 0;
    float prevItermPitch = 0;
    float desiredRatePitch = 0; // output from pid controller angle
    // roll
    float errorRoll = 0;
    float prevErrorRoll = 0;
    float prevItermRoll = 0;
    float desiredRateRoll = 0; // output from pid controller angle
    // rate pitch
    float errorRatePitch = 0;
    float prevErrorRatePitch = 0;
    float prevItermRatePitch = 0;
    float inputPitch = 0; // output from pid controller rate pitch
    // rate roll
    float errorRateRoll = 0;
    float prevErrorRateRoll = 0;
    float prevItermRateRoll = 0;
    float inputRoll = 0; // output from pid controller rate roll
    // rate yaw
    float errorRateYaw = 0;
    float prevErrorRateYaw = 0;
    float prevItermRateYaw = 0;
    float inputYaw = 0; // output from pid controller rate roll

    // motors speed
    int M_FR, M_BR, M_FL, M_BL;

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
        get_angle(fd, &gyroX, &gyroY, &gyroZ, &yaw, &kalmanPitch, &kalmanRoll,
                  accelPitchError, accelRollError, gyroPitchError, gyroRollError, gyroYawError,
                  &uncertantyPitch, &uncertantyRoll, &kalmanGainPitch, &kalmanGainRoll);
        printf(">[from main] pitch=%.2f roll=%.2f yaw=%.2f\n", kalmanPitch, kalmanRoll, yaw);
        errorRoll = desiredRoll - kalmanRoll;
        errorPitch = desiredPitch - kalmanPitch;
        pid_function(errorRoll, PAngleRoll, IAngleRoll, DAngleRoll, prevErrorRoll, prevItermRoll, -400, 400, PIDResult);
        desiredRateRoll = PIDReturn[0];
        prevErrorRoll = PIDReturn[1];
        prevItermRoll = PIDReturn[2];
        pid_function(errorPitch, PAnglePitch, IAnglePitch, DAnglePitch, prevErrorPitch, prevItermPitch, -400, 400, PIDResult);
        desiredRatePitch = PIDReturn[0];
        prevErrorPitch = PIDReturn[1];
        prevItermPitch = PIDReturn[2];
        errorRatePitch = desiredRatePitch - gyroX;
        errorRateRoll = desiredRateRoll - gyroY;
        errorRateYaw = desiredYaw - gyroZ;
        pid_equation(errorRateRoll, PRateRoll, IRateRoll, DRateRoll, prevErrorRateRoll, prevItermRateRoll, -400, 400, PIDResult);
        inputRoll = PIDReturn[0];
        prevErrorRateRoll = PIDReturn[1];
        prevItermRateRoll = PIDReturn[2];
        pid_equation(errorRatePitch, PRatePitch, IRatePitch, DRatePitch, prevErrorRatePitch, prevItermRatePitch, -400, 400, PIDResult);
        inputPitch = PIDReturn[0];
        prevErrorRatePitch = PIDReturn[1];
        prevItermRatePitch = PIDReturn[2];
        pid_equation(errorRateYaw, PRateYaw, IRateYaw, DRateYaw, prevErrorRateYaw, prevItermRateYaw, -400, 400, PIDResult);
        inputYaw = PIDReturn[0];
        prevErrorRateYaw = PIDReturn[1];
        prevItermRateYaw = PIDReturn[2];
        if (InputThrottle > 1800)
            InputThrottle = 1800;

        // mix input data
        M_FR = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
        M_BR = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
        M_BL = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
        M_FL = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

        // check maximum
        if (M_FR > 2000)
            M_FR = 1999;
        if (M_BR > 2000)
            M_BR = 1999;
        if (M_BL > 2000)
            M_BL = 1999;
        if (M_FL > 2000)
            M_FL = 1999;

        // check minimum
        if (M_FR < ThrottleMin)
            M_FR = ThrottleMin;
        if (M_BR < ThrottleMin)
            M_BR = ThrottleMin;
        if (M_FL < ThrottleMin)
            M_FL = ThrottleMin;
        if (M_BL < ThrottleMin)
            M_BL = ThrottleMin;
        printf("M_FR: %d M_FL: %d M_BR: %d M_BL: %d", M_FR, M_FL, M_BR, M_BL);
        while ((clock() - start) * 1000 / CLOCKS_PER_SEC < 4)
            ;
        start = clock();
    }
    return 0;
}