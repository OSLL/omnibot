#include <stdio.h>
#include "../../rw_control_global.h"
#include "../../rwheels.h"
#include "reflect.h"

#define RANGE_HISTORY_MAX (6*ECHO_SENSORS)
int rangeHistory[RANGE_HISTORY_MAX][2];
int historyHead = 0;

void reflectStart( callbackDataType* data ) {
    int ii;
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) {rangeHistory[ii][0] = 5; rangeHistory[ii][1] = ECHO_RANGE_CM_MAX;}
        serialPuts (seriald, ";;HELLO;;");
        serialPuts (seriald, "STOP,2;");
        serialPuts (seriald, "MODE,5,10,20,20;");
        serialPuts (seriald, "ECHO,15,0,500,7;");
//        serialPuts (seriald, "MODE,16,0,0,0;"); // Disable motors
        serialPuts (seriald, "MOVE,12,250,45,-126;");
        serialPuts (seriald, "DELTA,0,0,12,0,32000;");
    }

void reflectRange( callbackDataType* data ) {
    char buf[256];
    int ii;
    int minimum = 1;
    rangeHistory[historyHead][0] = data->range.sensor;
    rangeHistory[historyHead][1] = data->range.range;
    historyHead = ++historyHead % RANGE_HISTORY_MAX;
    if( historyHead == 0 ) { for( ii=0; ii<RANGE_HISTORY_MAX; ii++) { printf("%d ", rangeHistory[ii][1]); } printf("\n"); }
    if( data->range.range < 30 ) {
        printf("range: %d, sensor: %d\n", data->range.range, data->range.sensor);
        for( ii=0; ii<RANGE_HISTORY_MAX; ii++) { if( data->range.range > rangeHistory[ii][1] ) { minimum = 0; } }
        if( minimum ) {
            sprintf( buf, "MOVE,32001,32001,%d,32001;", ((data->range.sensor*90+360+45)+180)%360 ); 
            serialPuts( seriald, buf );
            serialPuts (seriald, "DELTA,0,0,12,0,32000;");
            printf( "%s\n", buf );
        }
    }
}

void reflectEmergency(callbackDataType* data) {
    serialPuts (seriald, "MODE,0,15,0,0;");
    serialPuts (seriald, "ECHO,15,0,0,20;");
    serialPuts (seriald, "STOP,2;");
    rwexit();
}

