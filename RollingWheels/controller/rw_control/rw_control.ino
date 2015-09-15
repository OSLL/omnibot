#include <limits.h>
#include "rw_control_global.h"
#include "rw_control.h"

/****************************************************/
/* Global variables and constants                   */
/****************************************************/
const int DEBUG = 0;

/****************************************************/
/* String table                                     */
/****************************************************/
const char String_Hello[] PROGMEM = STRING_HELLO;
const char String_Warn1[] PROGMEM = STRING_WARN1;
const char String_Warn2[] PROGMEM = STRING_WARN2;
const char String_Warn3[] PROGMEM = STRING_WARN3;
const char String_Error1[] PROGMEM = STRING_ERROR1;
const char String_Error2[] PROGMEM = STRING_ERROR2;
const char String_Error3[] PROGMEM = STRING_ERROR3;
const char String_Error4[] PROGMEM = STRING_ERROR4;
const char String_Error5[] PROGMEM = STRING_ERROR5;
const char String_Error6[] PROGMEM = STRING_ERROR6;
const char String_Error7[] PROGMEM = STRING_ERROR7;
const char String_Error8[] PROGMEM = STRING_ERROR8;
const char String_Error9[] PROGMEM = STRING_ERROR9;
const char String_Error10[] PROGMEM = STRING_ERROR10;
const char String_Error11[] PROGMEM = STRING_ERROR11;
const char String_Error12[] PROGMEM = STRING_ERROR12;
const char String_Error13[] PROGMEM = STRING_ERROR13;
const char String_Error14[] PROGMEM = STRING_ERROR14;
const char String_Error15[] PROGMEM = STRING_ERROR15;
const char* const string_table_local[] PROGMEM = {String_Hello};
const char* const string_table_warn[] PROGMEM = {String_Warn1, String_Warn2, String_Warn3};
const char* const string_table_error[] PROGMEM = {String_Error1, String_Error2, String_Error3, String_Error4, String_Error5, String_Error6, String_Error7, String_Error8,
                                                  String_Error9, String_Error10, String_Error11, String_Error12, String_Error13, String_Error14, String_Error15};

/****************************************************/
/* Input stream buffer and command data buffer      */
/****************************************************/
driveType bufCommand[COMMAND_BUF_LENGTH];
driveType *bufHead, *bufTail;
paramType paramBuf;

/****************************************************/
/* Mode                                             */
/****************************************************/
unsigned int currentMode = 0;
/****************************************************/
/* Motor time control                               */
/****************************************************/
unsigned long timer, setTime;
/****************************************************/
/* Last command values                              */
/****************************************************/
driveType lastCommand;
moveType lastMove = {0,0,0,0};
/****************************************************/
/* Echo sound control                               */
/****************************************************/
unsigned int currentRange = 0;
unsigned int echoRepeat = 0;
unsigned long echoRepeatTime;

/***###############################################################################***/

/****************************************************/
/* Initialization                                   */
/****************************************************/

void setup() {

  // Motor pins
  for( int ii=0; ii<4; ii++ )
  {
    digitalWrite(motorPin[ii], LOW);
    digitalWrite(motorPwmPin[ii], LOW);
    pinMode(motorPin[ii], OUTPUT);
    pinMode(motorPwmPin[ii], OUTPUT);
  }
  // Board reset for FW upgrade pin
  digitalWrite(forceResetPin, HIGH);
  pinMode( forceResetPin, OUTPUT);

  // Ultrasonic pins
//  digitalWrite(soundGroundPin, LOW);
  digitalWrite(soundPowerPin, HIGH);
  digitalWrite(soundTriggerPin, LOW);
//  pinMode( soundGroundPin, OUTPUT);
  pinMode( soundPowerPin, OUTPUT);
  pinMode( soundTriggerPin, OUTPUT);
  pinMode( soundEchoPin, INPUT);

  // Initialize serial:
  Serial.begin(115200);
  Serial.setTimeout(10);
  delay(1000);
  commandHello();

  // Initialize engines command queue and stop engines
  commandStop();
}

/****************************************************/
/* Command parcer                                   */
/****************************************************/

Ret_Status parceCommand(char* const buf) {
  Ret_Status ret;

  if( ! strncmp( buf, KeyDRIVE, strlen(KeyDRIVE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyDRIVE), sizeof(driveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    *bufHead = paramBuf.d;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyMOVE, strlen(KeyMOVE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyMOVE), sizeof(moveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processMoveParameters()) != RET_SUCCESS ) return ret;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
    lastMove = paramBuf.m;
  }
  else if( ! strncmp( buf, KeyDELTA, strlen(KeyDELTA) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyDELTA), sizeof(moveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processDeltaParameters()) != RET_SUCCESS ) return ret;
    if( (ret = processMoveParameters()) != RET_SUCCESS ) return ret;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
    lastMove = paramBuf.m;
  }
  else if( ! strncmp( buf, KeyROTATE, strlen(KeyROTATE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyROTATE), sizeof(rotateType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processRotateParameters()) != RET_SUCCESS ) return ret;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyMODE, strlen(KeyMODE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyMODE), sizeof(modeType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processModeParameters()) != RET_SUCCESS ) return ret;
  }
  else if (! strncmp( buf, KeyECHO, strlen(KeyECHO) )) {
    if( (ret = parceParameters( buf + strlen(KeyECHO), sizeof(echoType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processEchoParameters()) != RET_SUCCESS ) return ret;
  }
  else if (! strcmp( buf, KeySTOP )) { commandStop(); }
  else if (! strcmp( buf, KeySTATUS )) { commandStatus(); }
  else if (! strcmp( buf, KeyHELLO )) { commandHello(); }
  else if (! strcmp( buf, KeyEMPTY )) { }
  else { return RET_ERR_PARCE_COMMAND; }
  return RET_SUCCESS;
}

Ret_Status parceParameters ( char *head, int par_num ) {
  char *tail;
  long value;
  for( int ii=0; ii<par_num; ii++ )
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

Ret_Status queueCommand () {
    driveType *tempBufHead;
    Ret_Status ret;
    if( (ret = validateDriveParameters()) != RET_SUCCESS ) return ret;
    tempBufHead = bufHead + 1;
    if( tempBufHead == bufCommand + COMMAND_BUF_LENGTH ) { tempBufHead = bufCommand; }
    if( tempBufHead == bufTail) return RET_ERR_QUEUE_OVERLOAD;
    bufHead = tempBufHead;
    return RET_SUCCESS;
}

Ret_Status validateDriveParameters () {
  if( (bufHead->time > MAX_COMMAND_TIME) || (bufHead->time < 0)) return RET_ERR_PARAM_VALUE_TIME;
  for( int ii=0; ii<4; ii++ )
  {
    if( abs(bufHead->motor[ii]) > MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
  }
  return RET_SUCCESS;
}

Ret_Status processMoveParameters () {
  int angle_0_45, local_power, max_power;
  float course, rotation, correction, drive1, drive2;
  moveType *move = &(paramBuf.m);
  if( (abs(move->distance) > MAX_COMMAND_DISTANCE) && (abs(move->distance) < INFINITE_COMMAND) ) return RET_ERR_PARAM_VALUE_DISTANCE;
  if( abs(move->power) > MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
  if( abs(move->course) > MAX_COMMAND_COURSE) return RET_ERR_PARAM_VALUE_COURSE;
  if( abs(move->curve) > MAX_WAY_CURVE ) return RET_ERR_PARAM_VALUE_CURVE;

  if( abs(move->power) < calMove.min ) {
    move->power = 0;
    statusDecode(RET_WARN_MIN_POWER);
  }
  if( move->distance > INFINITE_COMMAND ) { move->distance = INFINITE_COMMAND; }
  if( move->distance < -INFINITE_COMMAND ) { move->distance = -INFINITE_COMMAND; }

  angle_0_45 = abs(move->course) % 90;
  if(angle_0_45 > 45) angle_0_45 = 90 - angle_0_45;
  course = move->course * CONST_PI / CONST_DEG_PER_PI;
  
  for( int jj=0; jj<10; jj++ ) {
    local_power = move->power;
    if ( move->distance < 0 ) local_power = -local_power;
    rotation = local_power * CAR_RADIUS * move->curve / 1000; // CarRadius / WayRadius
    
    correction = 1+angle_0_45/80.;
    drive1 = local_power*sin(course)*correction;
    drive2 = local_power*cos(course)*correction;
  
    bufHead->motor[0] = calibration(drive1, calMove) + calibration(drive1+rotation, calCurve) - calibration(drive1, calCurve);
    bufHead->motor[1] = calibration(drive2, calMove) + calibration(drive2+rotation, calCurve) - calibration(drive2, calCurve);
    bufHead->motor[2] = calibration(-drive1, calMove) + calibration(-drive1+rotation, calCurve) - calibration(-drive1, calCurve);
    bufHead->motor[3] = calibration(-drive2, calMove) + calibration(-drive2+rotation, calCurve) - calibration(-drive2, calCurve);

    max_power = 0;
    for( int ii=1; ii<4; ii++ ) { max_power = max(max_power, abs(bufHead->motor[ii])); }
    if( max_power > MAX_COMMAND_POWER ) {
      move->power *= (float)MAX_COMMAND_POWER / max_power;
      if(abs(local_power) == abs(move->power)) {
        if(move->power > 0) { (move->power)--; }
        if(move->power < 0) { (move->power)++; }
      }
      if( DEBUG ) {
        Serial.print( "Motor: " ); Serial.print( bufHead->motor[0] ); Serial.print( ' ' ); Serial.print( bufHead->motor[1] ); Serial.print( ' ' ); Serial.print( bufHead->motor[2] ); Serial.print( ' ' ); Serial.println( bufHead->motor[3] );
        Serial.print( "Power Reduced: " ); Serial.print( ' ' ); Serial.print( max_power ); Serial.print( ' ' ); Serial.print( move->power ); Serial.print( ' ' ); Serial.println( local_power );
      }
    } else {
      if( jj > 1 ) { statusDecode(RET_WARN_MAX_POWER_REACHED); }
      break;
    }
  }

  if( (move->power == 0) || (abs(move->distance) == INFINITE_COMMAND) )
  {
      bufHead->time = 0;
  } else {
      bufHead->time = 11000 * (long)abs(move->distance) / abs(local_power) / 4.3;
      if( bufHead->time == 0 ) {
          for( int ii=0; ii<4; ii++ ) { bufHead->motor[ii] = 0; }
      }
  }
  
  if( DEBUG ) {
    Serial.print( "Move Input: " ); Serial.print( move->distance ); Serial.print( ' ' ); Serial.print( move->power ); Serial.print( ' ' ); Serial.print( move->course ); Serial.print( ' ' ); Serial.println( move->curve );
    Serial.print( "Move1: " ); Serial.print( correction ); Serial.print( ' ' ); Serial.print( drive1 ); Serial.print( ' ' ); Serial.print( drive2 ); Serial.print( ' ' ); Serial.println( rotation );
    Serial.print( "Move2: " ); Serial.print( calibration(drive1, calMove) ); Serial.print( ' ' ); Serial.print( calibration(drive1+rotation, calCurve) ); Serial.print( ' ' ); Serial.println( calibration(drive1, calCurve) );
    Serial.print( "Move Output: " ); Serial.print( bufHead->time ); Serial.print( ' ' ); Serial.print( bufHead->motor[0] ); Serial.print( ' ' ); Serial.print( bufHead->motor[1] ); Serial.print( ' ' ); Serial.print( bufHead->motor[2] ); Serial.print( ' ' ); Serial.println( bufHead->motor[3] );
  }

  return RET_SUCCESS;
}

Ret_Status processDeltaParameters () {

    moveType *delta = &(paramBuf.m);

    if( (abs(delta->distance) > MAX_COMMAND_DISTANCE) && (abs(delta->distance) < INFINITE_COMMAND)) return RET_ERR_PARAM_VALUE_DISTANCE;
    if( abs(delta->power) > 2*MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
    if( abs(delta->course) > MAX_COMMAND_COURSE ) return RET_ERR_PARAM_VALUE_COURSE;
    if( abs(delta->curve) > 2*MAX_WAY_CURVE ) return RET_ERR_PARAM_VALUE_CURVE;

    if( delta->distance > INFINITE_COMMAND ) { delta->distance = INFINITE_COMMAND; }
    if( delta->distance < -INFINITE_COMMAND ) { delta->distance = -INFINITE_COMMAND; }

    if( abs(delta->distance) != INFINITE_COMMAND ) {
      if( abs(lastMove.distance) != INFINITE_COMMAND ) { delta->distance = lastMove.distance + delta->distance; }
      else { delta->distance = lastMove.distance; }
    }
    
    delta->power = lastMove.power + delta->power;
    delta->course = lastMove.course + delta->course;
    delta->curve = lastMove.curve + delta->curve;

    if( abs(lastMove.distance) != INFINITE_COMMAND ) {
        if (delta->distance > MAX_COMMAND_DISTANCE) { delta->distance = MAX_COMMAND_DISTANCE; }
        if (delta->distance < -MAX_COMMAND_DISTANCE) { delta->distance = -MAX_COMMAND_DISTANCE; }
    }
    if (delta->power > MAX_COMMAND_POWER) { delta->power = MAX_COMMAND_POWER; }
    if (delta->power < -MAX_COMMAND_POWER) { delta->power = -MAX_COMMAND_POWER; }
    delta->course = delta->course % 360;
    
    if( delta->curve > MAX_WAY_CURVE ) { delta->curve = MAX_WAY_CURVE; }
    if( delta->curve < -MAX_WAY_CURVE ) { delta->curve = -MAX_WAY_CURVE; }

    return RET_SUCCESS;  
}

Ret_Status processRotateParameters () {
    if (abs(paramBuf.r.power) > MAX_COMMAND_POWER) return RET_ERR_PARAM_VALUE_POWER;
    if (abs(paramBuf.r.power) < calRotation.min) {
      paramBuf.r.power = 0;
      statusDecode(RET_WARN_MIN_POWER);
    }
    if( paramBuf.r.angle > INFINITE_COMMAND ) { paramBuf.r.angle = INFINITE_COMMAND; }
    if( paramBuf.r.angle < -INFINITE_COMMAND ) { paramBuf.r.angle = -INFINITE_COMMAND; }

    int local_power = paramBuf.r.power;
    if ( paramBuf.r.angle < 0 ) local_power = -local_power;
    
    float fixed_power = calibration(local_power, calRotation);
    for( int ii=0; ii<4; ii++ ) { bufHead->motor[ii] = fixed_power; }
    
    if( (local_power == 0) || (abs(paramBuf.r.angle) == INFINITE_COMMAND) ) {
        bufHead->time = 0;
    } else {
        bufHead->time = ROTATION_MS_CM_PER_DEG * calRotation.factor * abs(paramBuf.r.angle) / abs(local_power);
        if( bufHead->time == 0 ) {
            for( int ii=0; ii<4; ii++ ) { bufHead->motor[ii] = 0; }
        }
    }

    if( DEBUG ) {
        Serial.print( "Rotate: " ); Serial.print( paramBuf.r.angle ); Serial.print( ' ' ); Serial.println( paramBuf.r.power );
        Serial.print( "Rotate: " ); Serial.print( bufHead->time ); Serial.print( ' ' ); Serial.println( bufHead->motor[0] );
    }

    return RET_SUCCESS;
}

Ret_Status processModeParameters () {
    currentMode |= paramBuf.o.set;
    currentMode &= ~paramBuf.o.reset;
    return RET_SUCCESS;
}

Ret_Status processEchoParameters () {
    if( 0 == paramBuf.e.repeat ) {
      echoRepeat = 0;
      commandEcho();
    }
    else if( paramBuf.e.repeat < MIN_ECHO_REPEAT ) {
      return RET_ERR_ECHO_REPEAT;
    }
    else {
      echoRepeat = paramBuf.e.repeat;
      echoRepeatTime = millis();
    }
    return RET_SUCCESS;
}

float calibration(float power, calibrationType cal) {
  float fixed_power;
  float a_factor = (1-cal.shift/(float)MAX_COMMAND_POWER);
  float y_turn = a_factor*cal.turn + cal.shift;
  
  if ( abs(power) < 1 ) { return 0; } // Avoid rounding error.
  if ( abs(power) < cal.turn ) {
    fixed_power = abs(power)*(y_turn-cal.cutoff)/cal.turn + cal.cutoff;
  } else {
    fixed_power = abs(power)*a_factor + cal.shift;
  }
  
  if( power > 0 ) { return fixed_power; }
  else { return -fixed_power; }
}

/***###############################################################################***/

/****************************************************/
/* Commands to execute or complete                  */
/****************************************************/

void commandDrive()
{
    for( int ii=0; ii<4; ii++ )
    {
        lastCommand.motor[ii] = bufTail->motor[ii];
        if( ! (currentMode & MODE_MOTOR_DISABLE) ) {
            if( bufTail->motor[ii] > 0 )
            {
              digitalWrite(motorPin[ii], HIGH);
              analogWrite(motorPwmPin[ii], MAX_COMMAND_POWER - bufTail->motor[ii]);
            } else {
              digitalWrite(motorPin[ii], LOW);
              analogWrite(motorPwmPin[ii], -(bufTail->motor[ii]));
            }
        }
    }
    timer = bufTail->time;
    lastCommand.time = bufTail->time;
    setTime = millis();
  
    bufTail++;
    if( bufTail == bufCommand + COMMAND_BUF_LENGTH ) { bufTail = bufCommand; }
}

void completeDrive()
{
  for( int ii=0; ii<4; ii++ )
  {
    digitalWrite(motorPin[ii], LOW);
    digitalWrite(motorPwmPin[ii], LOW);
    lastCommand.motor[ii] = 0;
  }
  timer = 0;
  lastCommand.time = 0;
  if( bufHead == bufTail) { commandStatus(); }
}

void commandStop()
{
    // Purge commands from circular queue
    bufHead = bufCommand;
    bufTail = bufCommand;
    // Stop current command immediately
    completeDrive();
}

void commandStatus()
{
  driveType *tempBufTail;
  unsigned long delta;
  unsigned long time_to_finish = 0;
  unsigned int queue = 0;

  // Calculate time to complete current command
  if( timer > 0 )
  {
    delta = (millis() - setTime);
    if(delta < timer) { time_to_finish += (timer - delta); }
  }

  // Look through all commands in the circular queue and add time from each of them
  tempBufTail = bufTail;
  while( tempBufTail != bufHead )
  {
    time_to_finish += tempBufTail->time;
    queue++;
    tempBufTail++;
    if( tempBufTail == bufCommand + COMMAND_BUF_LENGTH ) { tempBufTail = bufCommand; }
  }

  Serial.print(KeyREADY); Serial.print(KeyDELIMITER); Serial.print((int)(time_to_finish/CONST_MS_PER_SEC)); Serial.print(KeyDELIMITER); Serial.print(queue);
  Serial.print(KeyEOL1);

  Serial.print(KeyDRIVE); Serial.print(KeyDELIMITER); Serial.print(lastCommand.time);
  for( int ii=0; ii<4; ii++ ) { Serial.print(KeyDELIMITER); Serial.print(lastCommand.motor[ii]); }
  Serial.print(KeyEOL1);
  
  Serial.print(KeyLAST); Serial.print(KeyDELIMITER); Serial.print(lastMove.distance); Serial.print(KeyDELIMITER); Serial.print(lastMove.power); Serial.print(KeyDELIMITER); Serial.print(lastMove.course); Serial.print(KeyDELIMITER); Serial.print(lastMove.curve);
  Serial.print(KeyEOL1);

  Serial.print(KeyMODE); Serial.print(KeyDELIMITER); Serial.print(currentMode); Serial.print(KeyDELIMITER); Serial.print(COMMAND_BUF_LENGTH-1);
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);
}

void commandHello() {
  char line_buf[ MAX_STRING_LENGTH + 1 ];
  strcpy_P(line_buf, (char*)pgm_read_word(&(string_table_local[0])));
  Serial.print(KeyHELLO); Serial.print(KeyDELIMITER); Serial.println(line_buf);
}

void commandEcho() {
  long range = 0;

  digitalWrite(soundTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(soundTriggerPin, LOW);
  range = pulseIn(soundEchoPin, HIGH, (long)MAX_ECHO_RANGE_CM * SOUND_MS_PER_CM);
  currentRange = range / SOUND_MS_PER_CM;

  Serial.print(KeyECHO); Serial.print(KeyDELIMITER); Serial.print(currentRange);
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);
}

/***###############################################################################***/

/****************************************************/
/* Serial communication                             */
/****************************************************/

int readStream(char *buf, int len) {
  int bytes;
  bytes = Serial.readBytes(buf, len);
  if (bytes > 0)
  {
      buf[bytes] = 0;
      if( DEBUG ) { Serial.print("Bytes: "); Serial.print( bytes ); Serial.print(" "); Serial.println( buf ); }
      return RET_SUCCESS;
  }
  return RET_NO_COMMAND;
}

/****************************************************/
/* Errors                                           */
/****************************************************/

void statusDecode( Ret_Status ret )
{
  char line_buf[ MAX_STRING_LENGTH + 1 ];
  
  if ( (ret > RET_WARN_STATUS_START) && (ret < RET_ERR_STATUS_START) )
  {
    Serial.print(KeyWARN); Serial.print(KeyDELIMITER); Serial.print( ret ); Serial.print(KeyDELIMITER);
    if( ret > RET_WARN_UNKNOWN ) { ret = RET_WARN_UNKNOWN; }
    strcpy_P(line_buf, (char*)pgm_read_word(&(string_table_warn[ret - RET_WARN_STATUS_START - 1])));
    Serial.println(line_buf);
  }
  if ( ret > RET_ERR_STATUS_START )
  {
    Serial.print(KeyERROR); Serial.print(KeyDELIMITER); Serial.print( ret ); Serial.print(KeyDELIMITER);
    if( ret > RET_ERR_UNKNOWN ) { ret = RET_ERR_UNKNOWN; }
    strcpy_P(line_buf, (char*)pgm_read_word(&(string_table_error[ret - RET_ERR_STATUS_START - 1])));
    Serial.println(line_buf);
  }
}

/****************************************************/
/* Main Loop                                        */
/****************************************************/

void loop() {

  if( echoRepeat && (millis() > echoRepeatTime) ) {
    commandEcho();
    echoRepeatTime += echoRepeat; 
  }
  if( timer == 0 )
  {
      if( bufTail != bufHead ) { commandDrive(); }
  }
  else
  {
      if( millis() - setTime > timer )
      {
          if( bufTail != bufHead ) { commandDrive(); }
          else { completeDrive(); }
      }
  }
}

void serialEvent() {
  static char tempBuf[MAX_STREAM_LENGTH / 2];
  static char streamBuf[MAX_STREAM_LENGTH + 1] = {0};
  char *stream_start = streamBuf;
  char *command_end;

  if( readStream( streamBuf + strlen(streamBuf), MAX_STREAM_LENGTH - strlen(streamBuf) ) == RET_SUCCESS )
  {
    while( true )
    {
      // Reset board if FW upload detected
      if( (currentMode & MODE_FW_UPGRADE_ENABLE) && (*stream_start == '0') && (*(stream_start+1) == ' ') )
      {
        Serial.write(0x14);  // reply avrdude.exe
        Serial.write(0x10);  // for sync 
        digitalWrite(forceResetPin, LOW);  // reset the board
      }

      while( (command_end = strchr( stream_start, KeyEOL2 )) != NULL) { *command_end = KeyEOL1; }
      while( (command_end = strchr( stream_start, KeyEOL3 )) != NULL) { *command_end = KeyEOL1; }
      command_end = strchr( stream_start, KeyEOL1 );

      if( command_end != NULL )
      {
        *command_end = 0;
        if( DEBUG ) { Serial.print("To parce: "); Serial.println(stream_start); }
        statusDecode( parceCommand( stream_start ) );
        stream_start = command_end + 1;
      }
      else
      {
        if( strlen( stream_start ) >= MAX_STREAM_LENGTH / 2 )
        {
            streamBuf[0] = 0;
            statusDecode(RET_ERR_GARBAGE);
        }
        else
        {
            strcpy(tempBuf, stream_start );
            strcpy(streamBuf, tempBuf);
            if( DEBUG ) { Serial.print("Stream remainder: "); Serial.println(streamBuf); }
        }
        break;
      }
    }
  }
}

