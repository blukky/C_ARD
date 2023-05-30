#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <pigpio.h>
#include <unistd.h>

#define M_FL 26
#define M_FR 13
#define M_BR 6
#define M_BL 19


#define MIN_WIDTH 1000
#define MAX_WIDTH 2000

int run = 1;


void stop(int signum) {
    run = 0;
}


int limited(int a, float min, float max){
    if (a >= max){
        return max;
    }else if (a <= min){
        return min;
    }
    return a;
}


void setSpeedMotors(int motor, int speed){
    if (motor == 0){
        printf("all motors\n");
        gpioServo(M_FR, speed);
        gpioServo(M_FL, speed);
        gpioServo(M_BR, speed);
        gpioServo(M_BL, speed);        
    }else{
        gpioServo(motor, speed);
    }
}

float *pid_function(float k_p, float k_i, float k_d, float dt, float integral, float error, float last_error, float min, float max) {
    float out[3];
    integral = integral + error * dt;
    integral = limited(integral, min, max);
    float output = k_p * error + integral * k_i + k_d * ((error - last_error) / dt);
    output = limited(output, min, max);
    out[0] = output;
    out[1] = error;
    out[2] = integral;
    return out;
}

void calibrate(){
    int a;
    setSpeedMotors(0, 0);
    printf("Disconnect battery and press Enter\n");
    scanf("%d", &a);
    setSpeedMotors(0, MAX_WIDTH);
    printf("Connect battery and press Enter\n");
    scanf("%d", &a);
    setSpeedMotors(0, MIN_WIDTH);
    printf("strange, yse. Spetial tone\n");
    sleep(7);
    printf("Wait for it\n");
    sleep(5);
    printf("I working on this\n");
    setSpeedMotors(0, 0);
    sleep(2);
    printf("Enabling ESC now\n");
    setSpeedMotors(0, MIN_WIDTH);
    sleep(1);
    printf("See... Uhhhh");
}


void arm(){
    setSpeedMotors(0, 0);
    sleep(1);
    setSpeedMotors(0, MAX_WIDTH);
    sleep(1);
    setSpeedMotors(0, MIN_WIDTH);
    sleep(1);
}


void stop_motor(){
    setSpeedMotors(0,0);
    gpioTerminate();
}


int main(){
    if (gpioInitialise() < 0) return-1;
    gpioSetSignalFunc(SIGINT, stop);
    int ask;
    printf("Calibrate - 1, arm - 2\n");
    scanf("%d", &ask);
    if (ask == 1){
        calibrate();
    }else{
        arm();
    }

    int speed, motor;

    while(1){
        // printf("Set speed: ");
        // scanf("%d %d", &motor, &speed);
        // printf("\n");
        // speed = limited(speed);
        // setSpeedMotors(motor, speed);
    }
    stop_motor();
    return 0;

}