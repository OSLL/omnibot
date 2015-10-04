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
static void test1 (void);
static void test2 (void);
static void test3 (void);
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
        test3();
        serialReadLine();
    }

    serialClose(seriald);
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

#define RANGE_HISTORY_MAX (6*ECHO_SENSORS)
int rangeHistory[RANGE_HISTORY_MAX][2];
int historyHead = 0;
int flag_run = 1;

void test3(void) {
    static int first_run = 1;
    int ii;
    if( first_run ) {
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) {rangeHistory[ii][0] = 5; rangeHistory[ii][1] = MAX_ECHO_RANGE_CM;}
        serialPuts (seriald, ";;HELLO;;");
        serialPuts (seriald, "STOP;");
        serialPuts (seriald, "MODE,5,10,30,30;");
        serialPuts (seriald, "ECHO,15,0,500,7;");
//        serialPuts (seriald, "ECHO,16,0,0,0;"); // Disable motors
        serialPuts (seriald, "MOVE,10,210,45,-156;");
        serialPuts (seriald, "DELTA,0,0,12,0,32000;");
        first_run = 0;
    }
}

void callbackRange( int sensor, int range ) {
    char buf[256];
    int ii;
    int minimum = 1;
    rangeHistory[historyHead][0] = sensor;
    rangeHistory[historyHead][1] = range;
    historyHead = ++historyHead % RANGE_HISTORY_MAX;
    if( historyHead == 0 ) { for( ii=0; ii<RANGE_HISTORY_MAX; ii++) { printf("%d ", rangeHistory[ii][1]); } printf("\n"); }
    if( range < 30 ) {
        printf("range: %d, sensor: %d\n", range, sensor);
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) { if( range > rangeHistory[ii][1] ) { minimum = 0; } }
        if( minimum && flag_run ) {
            sprintf( buf, "MOVE,32001,32001,%d,32001;", ((sensor*90+360+45)+180)%360 ); 
            serialPuts( seriald, buf );
            serialPuts (seriald, "DELTA,0,0,12,0,32000;");
            printf( "%s\n", buf );
        }
    }
}

void callbackEmergency() {
    serialPuts (seriald, "MODE,0,15,0,0;");
    serialPuts (seriald, "ECHO,15,0,0,20;");
    flag_run = 0;
}

