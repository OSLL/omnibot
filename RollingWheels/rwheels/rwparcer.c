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
static Ret_Status parceParameters( char *head, int par_num );

/************************************************/
/*                                              */
/************************************************/

Ret_Status parceResponce( char* const buf ) {
    Ret_Status ret;
    if( DEBUG ) { printf( "Parcer input: %s\n", buf ); }

    if( ! strncmp( buf, KeyREADY, strlen(KeyREADY) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyREADY), sizeof(readyEvType)/sizeof(int) )) != RET_SUCCESS ) return ret;
        controllerData.ready = paramBuf.readyEv;
    }
    else if( ! strncmp( buf, KeyDRIVE, strlen(KeyDRIVE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyDRIVE), sizeof(driveEvType)/sizeof(int) )) != RET_SUCCESS ) return ret;
        controllerData.drive = paramBuf.driveEv;
    }
    else if( ! strncmp( buf, KeyECHO, strlen(KeyECHO) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyECHO), sizeof(echoEvType)/sizeof(int) )) != RET_SUCCESS ) return ret;
        controllerData.echo = paramBuf.echoEv;
    }
    else if( ! strncmp( buf, KeyMODE, strlen(KeyMODE) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyMODE), sizeof(modeEvType)/sizeof(int) )) != RET_SUCCESS ) return ret;
        controllerData.mode = paramBuf.modeEv;
    }
    else if( ! strncmp( buf, KeyLAST, strlen(KeyLAST) ))
    {
        if( (ret = parceParameters( buf + strlen(KeyMODE), sizeof(lastEvType)/sizeof(int) )) != RET_SUCCESS ) return ret;
        controllerData.last = paramBuf.lastEv;
    }
    else if( ! strncmp( buf, KeyHELLO, strlen(KeyHELLO) ))
    {
        printf( "Arduino: %s\n", buf );
    }
    else if( ! strncmp( buf, KeyERROR, strlen(KeyERROR) ))
    {
        printf( "Arduino: %s\n", buf );
    }
    else if( ! strncmp( buf, KeyWARN, strlen(KeyWARN) ))
    {
        printf( "Arduino: %s\n", buf );
    }
    else if( ! strcmp( buf, KeyEMPTY )) { return RET_SUCCESS; }
    else { return RET_ERR_PARCE_COMMAND; }

    if( DEBUG ) { printf( "Arduino: %s\n", buf ); }
    return RET_SUCCESS;
}

Ret_Status parceParameters( char *head, int par_num ) {
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
  if( *head != 0 ) return RET_ERR_PARCE_TERMINATOR;
  return RET_SUCCESS;
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
    printf("%s\n", string_table_warn[ret - RET_ERR_STATUS_START - 1]);
  }
  if ( ret > RET_ERR_STATUS_START )
  {
    printf("%s%c%d%c", KeyERROR, KeyDELIMITER, ret, KeyDELIMITER);
    if( ret > RET_ERR_UNKNOWN ) { ret = RET_ERR_UNKNOWN; }
    printf("%s\n", string_table_error[ret - RET_ERR_STATUS_START - 1]);
  }
}

