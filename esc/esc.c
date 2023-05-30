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
        speed = limited(speed, MIN_WIDTH, MAX_WIDTH);
        gpioServo(motor, speed);
    }
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