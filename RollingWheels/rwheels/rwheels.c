#include <wiringSerial.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "rw_control_global.h"
#include "rwheels.h"

/************************************************/
/* Funcion declarations                         */
/************************************************/
static void printCurrentData (void);
static void test1 (void);
static void test2 (void);
static void sleep_ms(int milliseconds);
static long long current_timestamp(void);

/************************************************/
/*                                              */
/************************************************/

char temp_str[256];

int main (void) {
    time_t tt = time(NULL);
    seriald = serialOpen ("/dev/ttyAMA0", 115200);

    while( 1 ) {
        test2();
        serialReadLine();
    }

    serialClose(seriald);
}

void printCurrentData (void) {
    printf("last: %d,%d,%d,%d; ", controllerData.last.distance, controllerData.last.velocity, controllerData.last.course, controllerData.last.curve);
    printf("ready: %d; ", controllerData.ready.queue);
    printf("drive: %d,%d,%d,%d,%d; ", controllerData.drive.time, controllerData.drive.motor[0], controllerData.drive.motor[1], controllerData.drive.motor[2], controllerData.drive.motor[3]);
    printf("mode: %d,%d; ", controllerData.mode.mode, controllerData.mode.queue);
    printf("echo: %d; ", controllerData.echo.range);
    printf("\n");
}

void sleep_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000 ) * 1000000;
    nanosleep(&ts, NULL);
}

long long current_timestamp(void) {
    struct timeval te;
    gettimeofday(&te, NULL);
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    return milliseconds;
}

void test2(void) {
    static long long time_ms;
    static int first_run = 1;
    int ii;
    if( first_run ) {
        time_ms = current_timestamp();
        serialPuts (seriald, ";;HELLO;;");
        serialPuts (seriald, "STOP;");
        serialPuts (seriald, "ECHO,400;");
        serialPuts (seriald, "MODE,1,0;");
//        serialPuts (seriald, "MOVE,3,59,45,-40;"); // Linear move with rotation
        serialPuts (seriald, "MOVE,3,59,45,0;");
        first_run = 0;
    }
    if(current_timestamp() - time_ms > 250) {
            printCurrentData();
            if (controllerData.ready.queue < (controllerData.mode.queue / 2)) {
                for(ii=0; ii<(controllerData.mode.queue / 2); ii++) {
                    serialPuts (seriald, "DELTA,0,0,10,0,0;");
                    sleep_ms(30);
                }
            }
            serialPuts (seriald, "STATUS;");
            time_ms = current_timestamp();
    }
}

void test1(void) {
    static time_t tt;
    static int first_run = 1;
    static int ttt = 200;
    if( first_run ) {
        tt = time(NULL);
        serialPuts (seriald, ";;HELLO;;");
        serialPuts (seriald, "ECHO,400;");
        serialPuts (seriald, "MODE,1,0;");
        first_run = 0;
    }
    if(time(NULL) - tt > 1) {
            serialPuts (seriald, "STATUS;");
            printCurrentData();
            if (controllerData.ready.queue < 5) {
                ttt++;
                sprintf( temp_str, "MOVE,%d,55,45,0;", ttt);
                serialPuts (seriald, temp_str);
            }
            tt = time(NULL);
    }
}

