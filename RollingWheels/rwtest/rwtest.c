#include <wiringSerial.h>
#include <stdio.h>
#include <string.h>
#include "rwheels.h"

#define MAX_STREAM_LENGTH 255

char streamBuf[MAX_STREAM_LENGTH + 1];
int seriald = -1;

/************************************************/
/* Funcion declarations                         */
/************************************************/

void serialReadLine( void );
int readStream( char *buf, int len );
void statusDecode( int ret );

/************************************************/
/*                                              */
/************************************************/

int main (void) {
    seriald = serialOpen ("/dev/ttyAMA0", 115200);

  while( 1 ) {
    serialPuts (seriald, ";;HELLO;");
//    serialPuts (seriald, "MOVE,50,140,45,0;");
//    serialPuts (seriald, "DRIVE,500,20,140,140,140;");
//    serialPuts (seriald, "MOVE,50,140,45,0;");
//    serialPuts (seriald, "DRIVE,500,20,140,140,140;");

        serialPuts (seriald, "STATUS;");
        sleep(5);
        serialReadLine();
    }

    serialClose(seriald);
}

void serialReadLine() {
    char tempBuf[MAX_STREAM_LENGTH / 2];
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
//              statusDecode( parceCommand( stream_start ) );
                printf( "Arduino: %s\n", stream_start );
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

void statusDecode( int ret )
{
    if ( ret > RET_ERR_STATUS_START )
    {
        printf("%s%c%d%c", KeyERROR, KeyDELIMITER, ret, KeyDELIMITER );
    } else {
        return;
    }
  
    switch( ret )
    {
      case RET_ERR_OVERLOAD :
          printf("Buffer Overload\n");
          break;
      case RET_ERR_PARCE_TERMINATOR :
          printf("No Terminator\n");
          break;
      case RET_ERR_PARCE_COMMAND :
          printf("Wrong Command\n");
          break;
      case RET_ERR_PARAM_NOT_FOUND :
          printf("No Parameters\n");
          break;
      case RET_ERR_PARAM_DELIMETER :
          printf("No Delimiter\n");
          break;
      case RET_ERR_GARBAGE :
          printf("Garbage in Stream Buffer\n");
          break;
      case RET_ERR_PARAM_VALUE_TIME :
          printf("Wrong Time Value\n");
          break;
      case RET_ERR_PARAM_VALUE_POWER :
          printf("Wrong Power Value\n");
          break;
      case RET_ERR_PARAM_VALUE_COURSE :
          printf("Wrong Course Value\n");
          break;
      case RET_ERR_PARAM_VALUE_RADIUS :
          printf("Wrong Radius Value\n");
          break;
      case RET_ERR_PARAM_VALUE_DISTANCE :
          printf("Wrong Distance Value\n");
          break;
      default:
          printf("Unknown Error\n");
          break;
    }
}
