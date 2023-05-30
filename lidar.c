#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <tof.h>


int main(){
    int i;
    int iDistance;   
    int model, version;
    
    i = tofInit(1, 0x29, 1);
    if (i != 1){
        return -1;
    }

    printf("VL53L0X Device succssfully opened.\n");

    i = tofGetModel(&model, &version);

    printf("Model ID - %d\n", model);
    printf("Version ID - %d\n", version);
    while (1)
    {
        iDistance  = tofReadDistance();
        if (iDistance < 4096){
            printf("Distance - %dmm\n", iDistance);
        }
        usleep(50000);
    }
    




    return 0;
}

