#include <wiringSerial.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rw_control_global.h"
#include "rwheels.h"
#include "rwtest.h"

/************************************************/
/* Funcion declarations                         */
/************************************************/
static void usage(void);

/************************************************/
/*                                              */
/************************************************/

int flag_exit = 0;

int main (int argc, char * argv[]) {
    int testNum = 0;
    int ii;
    for( ii=0; ii<CALLBACK_TERMINATOR; ii++ ) { callbacks[ii] = NULL; }

    if( argc < 2 ) {
        printf("Test name is not defined\n");
        usage();
        exit( 1 );
    }
    if( !strcmp( argv[1], "reflect" ) ) { testNum = 1; }
    
    switch( testNum ) {
        case 1:
            callbacks[CALLBACK_START] = reflectStart;
            callbacks[CALLBACK_RANGE] = reflectRange;
            callbacks[CALLBACK_EMERGENCY] = reflectEmergency;
            break;
        default:
            printf("Wrong test name\n");
            usage();
            exit( 2 );
    }

    seriald = serialOpen ("/dev/ttyAMA0", 115200);

    if( callbacks[CALLBACK_START] != NULL ) { callbacks[CALLBACK_START]( NULL ); }

    while( 1 ) {
        serialReadLine();
        if( flag_exit ) break;
    }

    serialClose(seriald);
    exit( 0 );
}

void usage(void) {
    printf("rwheels ver.0.001\n");
    printf("Usage: rwheels <testName>\n\n");
    printf("Available tests: reflect,...\n");
}

void rwexit() {
    flag_exit = 1;
}
