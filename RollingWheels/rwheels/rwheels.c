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
static void usage( int exitCode );

/************************************************/
/*                                              */
/************************************************/

static int flag_exit = 0;

int main (int argc, char * argv[]) {
    int ii;
    for( ii=0; ii<CALLBACK_TERMINATOR; ii++ ) { callbacks[ii] = NULL; }

    if( argc < 2 ) {
        printf("Test name is not defined\n");
        usage( 1 );
    }
    if( !strcmp( argv[1], "reflect" ) ) { reflectInit(); }
    else if( !strcmp( argv[1], "corr" ) ) { corrInit(); }
    else {
        printf("Wrong test name\n");
        usage( 2 );
    }
    
    seriald = serialOpen ("/dev/ttyAMA0", 115200);

    if( callbacks[CALLBACK_START] != NULL ) { callbacks[CALLBACK_START]( NULL ); }

    while( 1 ) {
        parcerReadLine();
        if( flag_exit ) break;
    }

    serialClose(seriald);
    exit( 0 );
}

void usage( int exitCode ) {
    printf("rwheels ver.0.001\n");
    printf("Usage: rwheels <testName>\n\n");
    printf("Available tests: reflect, corr,...\n");
    exit( exitCode );
}

void rwexit() {
    flag_exit = 1;
}
