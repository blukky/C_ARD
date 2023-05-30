#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <pigpio.h>
#include <unistd.h>



int run = 1;


void stop(int signum) {
    run = 0;
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