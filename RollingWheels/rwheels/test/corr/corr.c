#include <stdio.h>
#include "../../rw_control_global.h"
#include "../../rwheels.h"
#include "corr.h"

/************************************************/
/* Global data                                  */
/************************************************/
#define RANGE_HISTORY_MAX (300)
#define ECHO_DELAY (20)
static int rangeHistory[RANGE_HISTORY_MAX];
static int corrFunc[RANGE_HISTORY_MAX];
static long long timeStamp[RANGE_HISTORY_MAX];
static long long startTime;

/************************************************/
/* Funcion declarations                         */
/************************************************/
void corrRange( callbackDataType* data );
void corrEmergency( callbackDataType* data );
void corrStart( callbackDataType* data );
void corrFinished( void );

/************************************************/
/*                                              */
/************************************************/
void corrInit ( void ) {
    callbacks[CALLBACK_START] = corrStart;
    callbacks[CALLBACK_RANGE] = corrRange;
    callbacks[CALLBACK_EMERGENCY] = corrEmergency;
}

void corrStart( callbackDataType* data ) {
    int ii;
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) {rangeHistory[ii] = ECHO_RANGE_CM_MAX;}
        serialPuts (seriald, ";;HELLO;;");
        serialPuts (seriald, "STOP,2;");
//        serialPuts (seriald, "MODE,16,0,0,0;"); // Disable motors
        serialPuts (seriald, "MOVE,32000,60,45,500;");
        serialPuts (seriald, "MODE,1,10,20,20;"); // Use ECHO_DELAY
        serialPuts (seriald, "ECHO,15,0,500,7;");
        startTime = libCurrentTimestamp();
    }

void corrRange( callbackDataType* data ) {
    static int historyHead = 0;
    char buf[256];
    int ii, jj;
    long sum, fullTime;
    int num, corrMax, numMax, distance, theoryDistance, timePerCycle;
    int minimum = 1;
    rangeHistory[historyHead] = data->range.range;
    timeStamp[historyHead] = libCurrentTimestamp();
    historyHead = ++historyHead % RANGE_HISTORY_MAX;
    if( historyHead == 0 ) {
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) {
            printf("%d ", rangeHistory[ii]);
            if( ii%30 == 29 ) printf("\n");
        }
        corrFinished();
        fullTime = libCurrentTimestamp() - startTime;
        printf("Time = %ld ms\n", fullTime);
        for( ii=0; ii<RANGE_HISTORY_MAX/2; ii++) {
            sum = 0; num = 0;
            for( jj=ii; jj<RANGE_HISTORY_MAX; jj++ ) {
//                if( rangeHistory[jj-ii] != ECHO_RANGE_CM_MAX && rangeHistory[jj] != ECHO_RANGE_CM_MAX ) {
                    sum += rangeHistory[jj-ii]*rangeHistory[jj];
                    num++;
//                }
            }
            sum /= num;
            corrFunc[ii] = sum;
        }
        printf("\nCorrelation\n");
        for( ii=0; ii<RANGE_HISTORY_MAX/2; ii++) {
            printf("%d ", corrFunc[ii]);
            if( ii%30 == 29 ) printf("\n");
        }
        corrMax = 0; numMax = 0;
        for( ii=5; ii<RANGE_HISTORY_MAX/2; ii++) {
            if( corrMax < corrFunc[ii] ) {
                corrMax = corrFunc[ii];
                numMax = ii;
            }
        }
        theoryDistance = 10000/500*2*3.1415;
        printf("max = %d, num = %d, rotations = %f\n", corrMax, numMax, (float)RANGE_HISTORY_MAX/numMax);
//        printf("time per cycle experimental = %d, theory = %f\n", numMax*ECHO_DELAY, theoryDistance/80.*1000);
//        distance = (numMax*ECHO_DELAY)*80/1000;
//        sprintf( buf, "MOVE,%d,80,45,500;", distance);
//        timePerCycle = numMax*(fullTime/(float)RANGE_HISTORY_MAX);
        timePerCycle = timeStamp[numMax] - timeStamp[0] + 30;
        printf("Time per cycle = %d\n", timePerCycle);
        libSleepMs(2000);
        serialPuts(seriald, "MOVE,32000,60,45,500;");
        libSleepMs(timePerCycle);
        serialPuts(seriald, "STOP,2;");
        rwexit();
    }
}

void corrFinished( void ) {
    serialPuts (seriald, "MODE,0,15,0,0;");
    serialPuts (seriald, "ECHO,15,0,0,20;");
    serialPuts (seriald, "STOP,2;");
}

void corrEmergency(callbackDataType* data) {
    printf("Emergency exit\n");
    corrFinished();
    rwexit();
}

