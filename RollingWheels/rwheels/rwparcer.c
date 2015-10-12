#include <wiringSerial.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include "rw_control_global.h"
#include "rwheels.h"

#define MAX_STREAM_LENGTH 255

/************************************************/
/* Global variables                             */
/************************************************/
int seriald = -1;
paramType paramBuf;
controllerDataType controllerData;
void (*callbacks[CALLBACK_TERMINATOR])(callbackDataType* data);

/************************************************/
/* String table                                 */
/************************************************/
STRING_TABLE_GLOBAL

/************************************************/
/* Funcion declarations                         */
/************************************************/

static int readStream( char *buf, int len );
static void statusDecode( Ret_Status ret );
static Ret_Status parceResponce( char* const buf );
static Ret_Status parceParameters( char *head, int par_num, int string );
static void eventRange( int sensor, int range );
static void eventError( int error );
static void eventWarning( int warning );

/************************************************/
/* API                                          */
/************************************************/

void printCurrentData (void) {
    int ii;
    printf("move: %d,%d,%d,%d; ", controllerData.move.distance, controllerData.move.velocity, controllerData.move.course, controllerData.move.curve);
    printf("ready: %d,%d,%d; ", controllerData.ready.motion, controllerData.ready.queue, controllerData.ready.queueMax);
    printf("drive: %d,%d,%d,%d,%d; ", controllerData.drive.time, controllerData.drive.motor[0], controllerData.drive.motor[1], controllerData.drive.motor[2], controllerData.drive.motor[3]);
    printf("mode: %d,%d,%d;\n", controllerData.mode.mode, controllerData.mode.echoRepeat, controllerData.mode.echoNextSensor);
    for( ii=0; ii<ECHO_SENSORS; ii++ ) {
        printf("echo: %d,%d,%d,%d,%d; ", controllerData.echo[ii].sensor, controllerData.echo[ii].low, controllerData.echo[ii].high, controllerData.echo[ii].emergency, controllerData.echo[ii].range);
    }
    printf("\n");
}

/************************************************/
/* Parcer                                       */
/************************************************/

Ret_Status parceResponce( char* const buf ) {
    Ret_Status ret;
    if( DEBUG ) { printf( "Parcer input: %s\n", buf ); }

    if( ! strncmp( buf, KeyREADY, strlen(KeyREADY) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyREADY), sizeof(readyEvType)/sizeof(int), 0 )) != RET_SUCCESS ) return ret;
        controllerData.ready = paramBuf.readyEv;
    }
    else if( ! strncmp( buf, KeyDRIVE, strlen(KeyDRIVE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyDRIVE), sizeof(driveEvType)/sizeof(int), 0 )) != RET_SUCCESS ) return ret;
        controllerData.drive = paramBuf.driveEv;
    }
    else if( ! strncmp( buf, KeyECHO, strlen(KeyECHO) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyECHO), sizeof(echoEvType)/sizeof(int), 0 )) != RET_SUCCESS ) return ret;
        controllerData.echo[paramBuf.echoEv.sensor] = paramBuf.echoEv;
    }
    else if( ! strncmp( buf, KeyMODE, strlen(KeyMODE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyMODE), sizeof(modeEvType)/sizeof(int), 0 )) != RET_SUCCESS ) return ret;
        controllerData.mode = paramBuf.modeEv;
    }
    else if( ! strncmp( buf, KeyMOVE, strlen(KeyMOVE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyMOVE), sizeof(moveEvType)/sizeof(int), 0 )) != RET_SUCCESS ) return ret;
        controllerData.move = paramBuf.moveEv;
    }
    else if( ! strncmp( buf, KeyRANGE, strlen(KeyRANGE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyRANGE), sizeof(rangeEvType)/sizeof(int), 1 )) != RET_SUCCESS ) return ret;
        controllerData.echo[paramBuf.rangeEv.sensor].range = paramBuf.rangeEv.range;
        eventRange(paramBuf.rangeEv.sensor, paramBuf.rangeEv.range);
    }
    else if( ! strncmp( buf, KeyHELLO, strlen(KeyHELLO) ))
    {
        printf( "Arduino: %s\n", buf );
    }
    else if( ! strncmp( buf, KeyERROR, strlen(KeyERROR) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyERROR), sizeof(errorEvType)/sizeof(int), 1 )) != RET_SUCCESS ) return ret;
        eventError(paramBuf.errorEv.error);
    }
    else if( ! strncmp( buf, KeyWARN, strlen(KeyWARN) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyWARN), sizeof(errorEvType)/sizeof(int), 1 )) != RET_SUCCESS ) return ret;
        eventWarning(paramBuf.errorEv.error);
    }
    else if( ! strcmp( buf, KeyEMPTY )) { return RET_SUCCESS; }
    else { return RET_ERR_PARCE_COMMAND; }

    if( DEBUG ) { printf( "Arduino: %s\n", buf ); }
    return RET_SUCCESS;
}

Ret_Status parceParameters( char *head, int par_num, int string ) {
  char *tail;
  long value;
  int ii;
  for( ii=0; ii<par_num; ii++ )
  {
      if( *head != KeyDELIMITER ) return RET_ERR_PARAM_DELIMETER;
      head++;
      value = strtol( head, &tail, 10 );
      if( abs(value) <= INT_MAX ) { paramBuf.params[ii] = value; }
      else return RET_ERR_PARAM_VALUE_MAX;
      if( head == tail ) return RET_ERR_PARAM_NOT_FOUND;
      head = tail;
  }
  if( string && (*head == KeyDELIMITER) ) return RET_SUCCESS;
  if( *head != 0 ) return RET_ERR_PARCE_TERMINATOR;
  return RET_SUCCESS;
}

void eventRange( int sensor, int range ) {
    callbackDataType data;
    data.range.sensor = sensor;
    data.range.range = range;
    callbacks[CALLBACK_RANGE]( &data );
//    callbackRange( sensor, range );
}

void eventError( int error ) {
    printf( "Arduino: " );
    statusDecode( error );
}

void eventWarning( int warning ) {
    switch( warning ) {
        case RET_WARN_EMERGENCY_STOP:
            callbacks[CALLBACK_EMERGENCY](NULL);
            break;
    }
    printf( "Arduino: " );
    statusDecode( warning );
}

void serialReadLine() {
    static char streamBuf[MAX_STREAM_LENGTH + 1] = {0};
    static char tempBuf[MAX_STREAM_LENGTH / 2];
    char *stream_start = streamBuf;
    char *command_end;
    int bytes;

    if( readStream( streamBuf + strlen(streamBuf), MAX_STREAM_LENGTH - strlen(streamBuf) ) == RET_SUCCESS )
    {
        while( 1 )
        {
            while( (command_end = strchr( stream_start, KeyEOL2 )) != NULL) { *command_end = KeyEOL1; }
            while( (command_end = strchr( stream_start, KeyEOL3 )) != NULL) { *command_end = KeyEOL1; }
            command_end = strchr( stream_start, KeyEOL1 );

            if( command_end != NULL )
            {
                *command_end = 0;
                if( DEBUG ) { printf("To parce: %s\n", stream_start); }
                statusDecode( parceResponce( stream_start ) );
                stream_start = command_end + 1;
            }
            else
            {
                if( strlen( stream_start ) >= MAX_STREAM_LENGTH / 2 )
                {
                    streamBuf[0] = 0;
                    if( DEBUG ) { printf("Garbage detected\n"); }
                    statusDecode(RET_ERR_GARBAGE);
                }
                else
                {
                    strcpy(tempBuf, stream_start );
                    strcpy(streamBuf, tempBuf);
                    if( DEBUG ) { printf("Stream remainder: %s\n", streamBuf); }
                }
                break;
            }
        }
    }
}

int readStream(char *buf, int len) {
    int bytes;

    bytes = serialDataAvail( seriald );
    if( bytes <= 0 ) { return RET_NO_COMMAND; }
    if( bytes < len ) { len = bytes; }

    bytes = read( seriald, buf, len );
    if (bytes > 0)
    {
        buf[bytes] = 0;
        if( DEBUG ) { printf("Bytes: %d Line: %s\n", bytes, buf ); }
        return RET_SUCCESS;
    }
    return RET_NO_COMMAND;
}

/****************************************************/
/* Errors                                           */
/****************************************************/

void statusDecode( Ret_Status ret )
{
  if ( (ret > RET_WARN_STATUS_START) && (ret < RET_ERR_STATUS_START) )
  {
    printf("%s%c%d%c", KeyWARN, KeyDELIMITER, ret, KeyDELIMITER);
    if( ret > RET_WARN_UNKNOWN ) { ret = RET_WARN_UNKNOWN; }
    printf("%s\n", string_table_warn[ret - RET_WARN_STATUS_START - 1]);
  }
  if ( ret > RET_ERR_STATUS_START )
  {
    printf("%s%c%d%c", KeyERROR, KeyDELIMITER, ret, KeyDELIMITER);
    if( ret > RET_ERR_UNKNOWN ) { ret = RET_ERR_UNKNOWN; }
    printf("%s\n", string_table_error[ret - RET_ERR_STATUS_START - 1]);
  }
}

