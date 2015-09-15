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
const char String_Error16[] PROGMEM = STRING_ERROR16;
const char String_Error17[] PROGMEM = STRING_ERROR17;
const char* const string_table_local[] PROGMEM = {String_Hello};
const char* const string_table_warn[] PROGMEM = {String_Warn1, String_Warn2, String_Warn3};
const char* const string_table_error[] PROGMEM = {String_Error1, String_Error2, String_Error3, String_Error4, String_Error5, String_Error6, String_Error7, String_Error8,
                                                  String_Error9, String_Error10, String_Error11, String_Error12, String_Error13, String_Error14, String_Error15, String_Error16, String_Error17};

/****************************************************/
/* Test load                                        */
/****************************************************/
const char Test_Load1[] PROGMEM = "DRIVE,600,150,150,-150,-150";
const char Test_Load2[] PROGMEM = "DRIVE,600,150,0,-150,0";
const char Test_Load3[] PROGMEM = "MOVE,30,150,135,0";
const char Test_Load4[] PROGMEM = "DELTA,0,0,45,0,0";
const char Test_Load5[] PROGMEM = "ROTATE,800,120";
const char Test_Load6[] PROGMEM = "DRIVE,450,200,200,-200,-200";
const char Test_Load7[] PROGMEM = "DRIVE,450,200,0,-200,0";
const char Test_Load8[] PROGMEM = "MOVE,30,200,135,0";
const char Test_Load9[] PROGMEM = "DELTA,0,0,45,0,0";
const char Test_Load10[] PROGMEM = "MOVE,200,150,45,-30";
const char Test_Load11[] PROGMEM = "DELTA,-400,0,45,0,0";
const char Test_Load12[] PROGMEM = "MOVE,3,150,45,0";
const char Test_Load13[] PROGMEM = "DELTA,0,0,10,0,72";

const char* const test_table[] PROGMEM = {Test_Load1, Test_Load2, Test_Load3, Test_Load4, Test_Load5, Test_Load6, Test_Load7, Test_Load8, Test_Load9, Test_Load10, Test_Load11, Test_Load12, Test_Load13};
const int test_load_num = 13;

/****************************************************/
/* Input stream buffer and command data buffer      */
/****************************************************/
paramType bufCommand[COMMAND_BUF_LENGTH];
paramType *bufHead, *bufTail;
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
deltaType infiniteDelta = {0,0,0,0,0};
/****************************************************/
/* Echo sound control                               */
/****************************************************/
unsigned int currentRange = 0;
unsigned int echoRepeat = 0;
unsigned long echoRepeatTime;

/***######################################################################################################################################################################***/

/****************************************************/
/* Initialization                                   */
/****************************************************/

void setup() {

  char test_buf[ MAX_STRING_LENGTH + 1 ];
  int ii;
  Ret_Status ret;
  
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

  // Test load
  pinMode( testLoadPin, INPUT_PULLUP); // Shared with forceResetPin !!!!!
  if( digitalRead(testLoadPin) == LOW ) {
      for(ii=0; ii<test_load_num; ii++) {
        strcpy_P(test_buf, (char*)pgm_read_word(&(test_table[ii])));
        if( (ret = parceCommand(test_buf)) != RET_SUCCESS ) {
          statusDecode( ret );
        }
      }
  }

  // Board reset for FW upgrade pin
  digitalWrite(forceResetPin, HIGH);
  pinMode( forceResetPin, OUTPUT);
}

/***######################################################################################################################################################################***/

/****************************************************/
/* Command parcer                                   */
/****************************************************/

Ret_Status parceCommand(char* const buf) {
  Ret_Status ret;

  if( DEBUG ) { Serial.println(buf); }
  if( ! strncmp( buf, KeyDRIVE, strlen(KeyDRIVE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyDRIVE), sizeof(driveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateDriveParameters()) != RET_SUCCESS ) return ret;
    paramBuf.command = COMMAND_DRIVE;
    *bufHead = paramBuf;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyMOVE, strlen(KeyMOVE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyMOVE), sizeof(moveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateMoveParameters()) != RET_SUCCESS ) return ret;
    paramBuf.command = COMMAND_MOVE;
    *bufHead = paramBuf;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyDELTA, strlen(KeyDELTA) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyDELTA), sizeof(deltaType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateDeltaParameters()) != RET_SUCCESS ) return ret;
    paramBuf.command = COMMAND_DELTA;
    *bufHead = paramBuf;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyROTATE, strlen(KeyROTATE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyROTATE), sizeof(rotateType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateRotateParameters()) != RET_SUCCESS ) return ret;
    paramBuf.command = COMMAND_ROTATE;
    *bufHead = paramBuf;
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
    paramType *tempBufHead;
    Ret_Status ret;
    tempBufHead = bufHead + 1;
    if( tempBufHead == bufCommand + COMMAND_BUF_LENGTH ) { tempBufHead = bufCommand; }
    if( tempBufHead == bufTail) return RET_ERR_QUEUE_OVERLOAD;
    bufHead = tempBufHead;
    return RET_SUCCESS;
}

Ret_Status validateDriveParameters () {
  if( (paramBuf.d.time > MAX_COMMAND_TIME) || (paramBuf.d.time < 0)) return RET_ERR_PARAM_VALUE_TIME;
  for( int ii=0; ii<4; ii++ )
  {
    if( abs(paramBuf.d.motor[ii]) > MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
  }
  return RET_SUCCESS;
}

Ret_Status validateMoveParameters () {
  if( (abs(paramBuf.m.distance) > MAX_COMMAND_DISTANCE) && (abs(paramBuf.m.distance) < INFINITE_COMMAND) ) return RET_ERR_PARAM_VALUE_DISTANCE;
  if( abs(paramBuf.m.power) > MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
  if( abs(paramBuf.m.course) > MAX_COMMAND_COURSE) return RET_ERR_PARAM_VALUE_COURSE;
  if( abs(paramBuf.m.curve) > MAX_WAY_CURVE ) return RET_ERR_PARAM_VALUE_CURVE;

  if( paramBuf.m.distance > INFINITE_COMMAND ) { paramBuf.m.distance = INFINITE_COMMAND; }
  if( paramBuf.m.distance < -INFINITE_COMMAND ) { paramBuf.m.distance = -INFINITE_COMMAND; }
  return RET_SUCCESS;
}

Ret_Status validateDeltaParameters () {
    if( paramBuf.md.repeat < 0) return RET_ERR_PARAM_VALUE_REPEAT;
    if( (abs(paramBuf.md.distance) > MAX_COMMAND_DISTANCE) && (abs(paramBuf.md.distance) < INFINITE_COMMAND)) return RET_ERR_PARAM_VALUE_DISTANCE;
    if( abs(paramBuf.md.power) > 2*MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
    if( abs(paramBuf.md.course) > MAX_COMMAND_COURSE ) return RET_ERR_PARAM_VALUE_COURSE;
    if( abs(paramBuf.md.curve) > 2*MAX_WAY_CURVE ) return RET_ERR_PARAM_VALUE_CURVE;
    
    if( paramBuf.md.distance > INFINITE_COMMAND ) { paramBuf.md.distance = INFINITE_COMMAND; }
    if( paramBuf.md.distance < -INFINITE_COMMAND ) { paramBuf.md.distance = -INFINITE_COMMAND; }
    if( paramBuf.md.repeat > INFINITE_COMMAND) { paramBuf.md.repeat = INFINITE_COMMAND; }
    return RET_SUCCESS;
}

Ret_Status validateRotateParameters () {
    if (abs(paramBuf.r.power) > MAX_COMMAND_POWER) return RET_ERR_PARAM_VALUE_POWER;
    if (abs(paramBuf.r.power) < calRotation.min) {
      paramBuf.r.power = 0;
      statusDecode(RET_WARN_MIN_POWER);
    }
    if( paramBuf.r.angle > INFINITE_COMMAND ) { paramBuf.r.angle = INFINITE_COMMAND; }
    if( paramBuf.r.angle < -INFINITE_COMMAND ) { paramBuf.r.angle = -INFINITE_COMMAND; }
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

/***######################################################################################################################################################################***/

/****************************************************/
/* Convert command to motor and time values         */
/****************************************************/
void commandPrepare() {
    moveType move;
    if( DEBUG ) {
        Serial.print( "queue: " ); Serial.print( bufTail->params[0] ); Serial.print( ' ' ); Serial.print( bufTail->params[1] ); Serial.print( ' ' ); Serial.print( bufTail->params[2] );
        Serial.print( ' ' ); Serial.print( bufTail->params[3] ); Serial.print( ' ' ); Serial.print( bufTail->params[4] ); Serial.print( ' ' ); Serial.println( bufTail->command );
    }

    infiniteDelta = {0,0,0,0,0};

    switch( bufTail->command ) {
      case COMMAND_DRIVE:
        lastCommand = bufTail->d;
        commandDrive();
        break;
      case COMMAND_MOVE:
        processMoveParameters(&(bufTail->m));
        commandDrive();
        lastMove = bufTail->m;
        break;
      case COMMAND_DELTA:
        move = bufTail->m;
        processDeltaParameters(&move);
        processMoveParameters(&move);
        commandDrive();
        lastMove = move;
        if( (bufTail->md.repeat) == INFINITE_COMMAND ) {
            infiniteDelta = bufTail->md;
        } else {
            if( bufTail->md.repeat > 0 ) {
                (bufTail->md.repeat)--;
                return; // Keep the delta command in the queue while repeate > 0
            }
        }
        break;
      case COMMAND_ROTATE:
        processRotateParameters();
        commandDrive();
        break;
      default:
        statusDecode( RET_ERR_SYSTEM_CRITICAL );
        commandStop();
        return;
    }
    bufTail++;
    if( bufTail == bufCommand + COMMAND_BUF_LENGTH ) { bufTail = bufCommand; }
}

Ret_Status processInfiniteDelta() {
    deltaType delta;
    moveType* move;
    if( infiniteDelta.repeat == INFINITE_COMMAND ) {
        delta = infiniteDelta;
        move = (moveType*)(&delta);
        processDeltaParameters(move);
        processMoveParameters(move);
        commandDrive();
        lastMove = *move;
        return RET_SUCCESS;
    } else {
        return RET_NO_INFINITE_DELTA;
    }
}

void processRotateParameters () {

    int local_power = bufTail->r.power;
    if ( bufTail->r.angle < 0 ) local_power = -local_power;
    
    float fixed_power = calibration(local_power, calRotation);
    for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = fixed_power; }
    
    if( (local_power == 0) || (abs(bufTail->r.angle) == INFINITE_COMMAND) ) {
        lastCommand.time = 0;
    } else {
        //TODO time can exceed MAX possible value. Need to check at validate stage
        lastCommand.time = ROTATION_MS_CM_PER_DEG * calRotation.factor * abs(bufTail->r.angle) / abs(local_power);
        if( lastCommand.time == 0 ) {
            for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
        }
    }

    if( DEBUG ) {
        Serial.print( "Rotate: " ); Serial.print( bufTail->r.angle ); Serial.print( ' ' ); Serial.println( bufTail->r.power );
        Serial.print( "Rotate: " ); Serial.print( lastCommand.time ); Serial.print( ' ' ); Serial.println( lastCommand.motor[0] );
    }
}

void processMoveParameters( moveType* mv ) {
  int angle_0_45, local_power, max_power;
  float course, rotation, correction, drive1, drive2;

  if( abs(mv->power) < calMove.min ) {
    mv->power = 0;
    statusDecode(RET_WARN_MIN_POWER);
  }

  angle_0_45 = abs(mv->course) % 90;
  if(angle_0_45 > 45) angle_0_45 = 90 - angle_0_45;
  course = mv->course * CONST_PI / CONST_DEG_PER_PI;
  
  for( int jj=0; jj<10; jj++ ) {
    //TODO process error if jj=10 is not enough and power still more than MAX
    local_power = mv->power;
    if ( mv->distance < 0 ) local_power = -local_power;
    rotation = local_power * CAR_RADIUS * mv->curve / 1000; // CarRadius / WayRadius
    
    correction = 1+angle_0_45/80.;
    drive1 = local_power*sin(course)*correction;
    drive2 = local_power*cos(course)*correction;
  
    lastCommand.motor[0] = calibration(drive1, calMove) + calibration(drive1+rotation, calCurve) - calibration(drive1, calCurve);
    lastCommand.motor[1] = calibration(drive2, calMove) + calibration(drive2+rotation, calCurve) - calibration(drive2, calCurve);
    lastCommand.motor[2] = calibration(-drive1, calMove) + calibration(-drive1+rotation, calCurve) - calibration(-drive1, calCurve);
    lastCommand.motor[3] = calibration(-drive2, calMove) + calibration(-drive2+rotation, calCurve) - calibration(-drive2, calCurve);

    max_power = 0;
    for( int ii=1; ii<4; ii++ ) { max_power = max(max_power, abs(lastCommand.motor[ii])); }
    if( max_power > MAX_COMMAND_POWER ) {
      mv->power *= (float)MAX_COMMAND_POWER / max_power;
      if(abs(local_power) == abs(mv->power)) {
        if(mv->power > 0) { (mv->power)--; }
        if(mv->power < 0) { (mv->power)++; }
      }
      if( DEBUG ) {
        Serial.print( "Motor: " ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
        Serial.print( "Power Reduced: " ); Serial.print( ' ' ); Serial.print( max_power ); Serial.print( ' ' ); Serial.print( mv->power ); Serial.print( ' ' ); Serial.println( local_power );
      }
    } else {
      if( jj > 1 ) { statusDecode(RET_WARN_MAX_POWER_REACHED); }
      break;
    }
  }

  if( (mv->power == 0) || (abs(mv->distance) == INFINITE_COMMAND) )
  {
      lastCommand.time = 0;
  } else {
      //TODO time can exceed MAX possible value. Need to check at validate stage
      lastCommand.time = 11000 * (long)abs(mv->distance) / abs(local_power) / 4.3;
      if( lastCommand.time == 0 ) {
          for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
      }
  }
  
  if( DEBUG ) {
    Serial.print( "Move Input: " ); Serial.print( mv->distance ); Serial.print( ' ' ); Serial.print( mv->power ); Serial.print( ' ' ); Serial.print( mv->course ); Serial.print( ' ' ); Serial.println( mv->curve );
    Serial.print( "Move1: " ); Serial.print( correction ); Serial.print( ' ' ); Serial.print( drive1 ); Serial.print( ' ' ); Serial.print( drive2 ); Serial.print( ' ' ); Serial.println( rotation );
    Serial.print( "Move2: " ); Serial.print( calibration(drive1, calMove) ); Serial.print( ' ' ); Serial.print( calibration(drive1+rotation, calCurve) ); Serial.print( ' ' ); Serial.println( calibration(drive1, calCurve) );
    Serial.print( "Move Output: " ); Serial.print( lastCommand.time ); Serial.print( ' ' ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
  }
}

void processDeltaParameters( moveType* dm ) {

    if( abs(dm->distance) != INFINITE_COMMAND ) {
      if( abs(lastMove.distance) != INFINITE_COMMAND ) { dm->distance = lastMove.distance + dm->distance; }
      else { dm->distance = lastMove.distance; }
    }
    if( abs(dm->distance) != INFINITE_COMMAND ) {
        if (dm->distance > MAX_COMMAND_DISTANCE) { dm->distance = MAX_COMMAND_DISTANCE; }
        if (dm->distance < -MAX_COMMAND_DISTANCE) { dm->distance = -MAX_COMMAND_DISTANCE; }
    }
    
    dm->power = lastMove.power + dm->power;
    if (dm->power > MAX_COMMAND_POWER) { dm->power = MAX_COMMAND_POWER; }
    if (dm->power < -MAX_COMMAND_POWER) { dm->power = -MAX_COMMAND_POWER; }

    dm->course = lastMove.course + dm->course;
    dm->course = dm->course % MAX_COMMAND_COURSE;

    dm->curve = lastMove.curve + dm->curve;
    if( dm->curve > MAX_WAY_CURVE ) { dm->curve = MAX_WAY_CURVE; }
    if( dm->curve < -MAX_WAY_CURVE ) { dm->curve = -MAX_WAY_CURVE; }
}

/***######################################################################################################################################################################***/

/****************************************************/
/* Commands to execute or complete                  */
/****************************************************/

void commandDrive()
{
    if( (lastCommand.time > MAX_COMMAND_TIME) || (lastCommand.time < 0)) {
      statusDecode( RET_ERR_PARAM_VALUE_TIME );
    } else {
      for( int ii=0; ii<4; ii++ )
      {
        if( ! (currentMode & MODE_MOTOR_DISABLE) ) {
            if( lastCommand.motor[ii] > 0 )
            {
              digitalWrite(motorPin[ii], HIGH);
              analogWrite(motorPwmPin[ii], MAX_COMMAND_POWER - lastCommand.motor[ii]);
            } else {
              digitalWrite(motorPin[ii], LOW);
              analogWrite(motorPwmPin[ii], -(lastCommand.motor[ii]));
            }
        }
      }
      timer = lastCommand.time;
      setTime = millis();
      if( DEBUG ) {
        Serial.print( "commandDrive: " ); Serial.print( lastCommand.time ); Serial.print( ' ' ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] );
        Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
      }
    }    
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
    // Purge infinite delta
    infiniteDelta = {0,0,0,0,0};
    // Stop current command immediately
    completeDrive();
}

void commandStatus()
{
  paramType *tempBufTail;
  unsigned int queue = 0;

  // Look through all commands in the circular queue and add time from each of them
  tempBufTail = bufTail;
  while( tempBufTail != bufHead )
  {
    queue++;
    tempBufTail++;
    if( tempBufTail == bufCommand + COMMAND_BUF_LENGTH ) { tempBufTail = bufCommand; }
  }

  Serial.print(KeyREADY); Serial.print(KeyDELIMITER); Serial.print(queue);
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

/***######################################################################################################################################################################***/

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
      if( bufTail != bufHead ) { commandPrepare(); }
      else { processInfiniteDelta(); }
  }
  else
  {
      if( millis() - setTime > timer )
      {
          if( bufTail != bufHead ) {
              commandPrepare();
          } else {
              if( processInfiniteDelta() != RET_SUCCESS ) { completeDrive(); }
          }
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

