#include <limits.h>
#include "rw_control_global.h"
#include "rw_control.h"

/****************************************************/
/* Global variables and constants                   */
/****************************************************/
const int DEBUG = 0;

/****************************************************/
/* String tables                                    */
/****************************************************/
STRING_TABLE_GLOBAL

const char String_Hello[] STRING_MEM_MODE = "Rolling Wheels Ard. ver:0.032 (keep)";
const char* const string_table_local[] STRING_MEM_MODE = {String_Hello};

/****************************************************/
/* Test load                                        */
/****************************************************/
const char Test_Load1[] PROGMEM = "DRIVE,600,150,150,-150,-150";
const char Test_Load2[] PROGMEM = "DRIVE,600,150,0,-150,0";
const char Test_Load3[] PROGMEM = "MOVE,30,59,135,0";
const char Test_Load4[] PROGMEM = "DELTA,0,0,45,0,0";
const char Test_Load5[] PROGMEM = "ROTATE,800,120";
const char Test_Load6[] PROGMEM = "DRIVE,450,200,200,-200,-200";
const char Test_Load7[] PROGMEM = "DRIVE,450,200,0,-200,0";
const char Test_Load8[] PROGMEM = "MOVE,30,78,135,0";
const char Test_Load9[] PROGMEM = "DELTA,0,0,45,0,0";
const char Test_Load10[] PROGMEM = "MOVE,200,59,45,-30";
const char Test_Load11[] PROGMEM = "DELTA,-400,0,45,0,0";
const char Test_Load12[] PROGMEM = "MOVE,3,59,45,0";
const char Test_Load13[] PROGMEM = "DELTA,0,0,10,0,72";

const char* const test_table[] PROGMEM = {Test_Load1, Test_Load2, Test_Load3, Test_Load4, Test_Load5, Test_Load6, Test_Load7, Test_Load8, Test_Load9, Test_Load10, Test_Load11, Test_Load12, Test_Load13};
const int test_load_num = 13;

/****************************************************/
/* Input stream buffer and command data buffer      */
/****************************************************/
paramType bufCommand[COMMAND_BUF_LENGTH];
paramType *bufHead, *bufTail;

/****************************************************/
/* Mode                                             */
/****************************************************/
unsigned int currentMode = 0;
/****************************************************/
/* Motor time control                               */
/****************************************************/
unsigned long timer, setTime;
int motionInProgress = 0;
/****************************************************/
/* Last command values                              */
/****************************************************/
driveType lastCommand = {0,0,0,0,0};
moveType lastMove = {0,0,0,0};
deltaType infiniteDelta = {0,0,0,0,0};
/****************************************************/
/* Echo sound control                               */
/****************************************************/
echoType echoConfig[ECHO_SENSORS] = {MAX_ECHO_RANGE_CM,0,0,0, MAX_ECHO_RANGE_CM,0,0,0, MAX_ECHO_RANGE_CM,0,0,0, MAX_ECHO_RANGE_CM,0,0,0};
unsigned long echoRepeatTime;
int echoRepeat = MIN_ECHO_REPEAT;
int echoNextSensor = MIN_ECHO_REPEAT;
unsigned char echoSensor = 0;
unsigned long volatile isrEchoTime[ECHO_ISR_LAST];
unsigned char volatile isrEchoIndex = ECHO_ISR_COMPLETE;

/***######################################################################################################################################################################***/

/****************************************************/
/* Initialization                                   */
/****************************************************/

void setup() {

  char test_buf[ MAX_STRING_LENGTH + 1 ];
  int ii;
  Ret_Status ret;
  
  // Motor pins
  for( ii=0; ii<4; ii++ ) {
    digitalWrite(motorPin[ii], LOW);
    digitalWrite(motorPwmPin[ii], LOW);
    pinMode(motorPin[ii], OUTPUT);
    pinMode(motorPwmPin[ii], OUTPUT);
  }

  // Ultrasonic pins
  for( ii=0; ii<ECHO_SENSORS; ii++ ) {
      digitalWrite(soundTriggerPin[ii], LOW);
      pinMode( soundTriggerPin[ii], OUTPUT);
      pinMode( soundEchoPin[ii], INPUT);
  }

  // Initialize serial:
  Serial.begin(115200);
  Serial.setTimeout(10);
  delay(1000);
  commandHello();

  // Initialize engines command queue and stop engines
  commandStop();

  // Test load
  pinMode( sharedTestResetPin, INPUT_PULLUP);
  if( digitalRead(sharedTestResetPin) == LOW ) {
      for(ii=0; ii<test_load_num; ii++) {
        strcpy_P(test_buf, (char*)pgm_read_word(&(test_table[ii])));
        if( (ret = parceCommand(test_buf)) != RET_SUCCESS ) {
          statusDecode( ret );
        }
      }
  }

  // Board reset for FW upgrade pin
  digitalWrite(sharedTestResetPin, HIGH);
  pinMode( sharedTestResetPin, OUTPUT);
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
    bufHead->command = COMMAND_DRIVE;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyMOVE, strlen(KeyMOVE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyMOVE), sizeof(moveType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateMoveParameters()) != RET_SUCCESS ) return ret;
    bufHead->command = COMMAND_MOVE;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyDELTA, strlen(KeyDELTA) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyDELTA), sizeof(deltaType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateDeltaParameters()) != RET_SUCCESS ) return ret;
    bufHead->command = COMMAND_DELTA;
    if( (ret = queueCommand()) != RET_SUCCESS ) return ret;
  }
  else if( ! strncmp( buf, KeyROTATE, strlen(KeyROTATE) ))
  {
    if( (ret = parceParameters( buf + strlen(KeyROTATE), sizeof(rotateType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = validateRotateParameters()) != RET_SUCCESS ) return ret;
    bufHead->command = COMMAND_ROTATE;
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
      if( abs(value) <= INT_MAX ) { bufHead->params[ii] = value; }
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
  if( (bufHead->drive.time > MAX_COMMAND_TIME) || (bufHead->drive.time < 0)) return RET_ERR_PARAM_VALUE_TIME;
  for( int ii=0; ii<4; ii++ )
  {
    if( abs(bufHead->drive.motor[ii]) > MAX_COMMAND_POWER ) return RET_ERR_PARAM_VALUE_POWER;
  }
  return RET_SUCCESS;
}

Ret_Status validateMoveParameters () {
  if( (abs(bufHead->move.distance) > MAX_MOVE_DISTANCE) && (abs(bufHead->move.distance) != INFINITE_COMMAND)
                                                        && (bufHead->move.distance != KEEP_PARAMETER) ) return RET_ERR_PARAM_VALUE_DISTANCE;
  if( (abs(bufHead->move.velocity) > MAX_MOVE_VELOCITY) && (bufHead->move.velocity != KEEP_PARAMETER) ) return RET_ERR_PARAM_VALUE_POWER;
  if( (abs(bufHead->move.course) > MAX_MOVE_COURSE) && (bufHead->move.course != KEEP_PARAMETER) ) return RET_ERR_PARAM_VALUE_COURSE;
  if( (abs(bufHead->move.curve) > MAX_MOVE_CURVE) && (bufHead->move.curve != KEEP_PARAMETER) ) return RET_ERR_PARAM_VALUE_CURVE;

  return RET_SUCCESS;
}

Ret_Status validateDeltaParameters () {
    if( (bufHead->delta.repeat < 0) || (bufHead->delta.repeat > INFINITE_COMMAND) ) return RET_ERR_PARAM_VALUE_REPEAT;
    if( (abs(bufHead->delta.distance) > MAX_MOVE_DISTANCE /*TBD*/) && (abs(bufHead->delta.distance) != INFINITE_COMMAND)) return RET_ERR_PARAM_VALUE_DISTANCE;
    if( abs(bufHead->delta.velocity) > 2*MAX_MOVE_VELOCITY ) return RET_ERR_PARAM_VALUE_POWER;
    if( abs(bufHead->delta.course) > MAX_MOVE_COURSE ) return RET_ERR_PARAM_VALUE_COURSE;
    if( abs(bufHead->delta.curve) > 2*MAX_MOVE_CURVE ) return RET_ERR_PARAM_VALUE_CURVE;
    
    return RET_SUCCESS;
}

Ret_Status validateRotateParameters () {
    if (abs(bufHead->rotate.power) > MAX_COMMAND_POWER) return RET_ERR_PARAM_VALUE_POWER;
    if (abs(bufHead->rotate.angle) > INFINITE_COMMAND) return RET_ERR_PARAM_VALUE_ANGLE;
    if (abs(bufHead->rotate.power) < MIN_POWER_ROTATION) {
      bufHead->rotate.power = 0;
      statusDecode(RET_WARN_MIN_POWER);
    }
    return RET_SUCCESS;  
}

Ret_Status processModeParameters () {
    unsigned int oldMode = currentMode;
    if( (bufHead->mode.echoRepeat != 0) && (bufHead->mode.echoRepeat < MIN_ECHO_REPEAT) ) return RET_ERR_ECHO_REPEAT;
    if( (bufHead->mode.echoNextSensor != 0) && (bufHead->mode.echoNextSensor < MIN_ECHO_REPEAT) ) return RET_ERR_ECHO_REPEAT;
    
    currentMode |= bufHead->mode.set;
    currentMode &= ~(bufHead->mode.reset);

    // If all echo sensors were disabled and any sensor is enabling then set echoRepeatTime to current time
    if( !(oldMode & (MODE_ECHO_0 | MODE_ECHO_1 | MODE_ECHO_2 | MODE_ECHO_3)) ) {
        if( currentMode & (MODE_ECHO_0 | MODE_ECHO_1 | MODE_ECHO_2 | MODE_ECHO_3) ) { echoRepeatTime = millis(); }
    }
    if( bufHead->mode.echoRepeat != 0 ) { echoRepeat = bufHead->mode.echoRepeat; }
    if( bufHead->mode.echoNextSensor != 0 ) { echoNextSensor = bufHead->mode.echoNextSensor; }
    
    return RET_SUCCESS;
}

Ret_Status processEchoParameters () {
    int ii;
    if( (bufHead->echo.low < 0) || (bufHead->echo.low > MAX_ECHO_RANGE_CM) ) return RET_ERR_ECHO_DISTANCE;
    if( (bufHead->echo.high < 0) || (bufHead->echo.high > MAX_ECHO_RANGE_CM) ) return RET_ERR_ECHO_DISTANCE;
    if( (bufHead->echo.emergency < 0) || (bufHead->echo.emergency > MAX_ECHO_RANGE_CM) ) return RET_ERR_ECHO_DISTANCE;
    bufHead->echo.sensors &= 0x000F;
    
    for( ii=0; ii<ECHO_SENSORS; ii++ ) {
      if( bufHead->echo.sensors & (0x0001 << ii) ) {
        echoConfig[ii].low = bufHead->echo.low;
        echoConfig[ii].high = bufHead->echo.high;
        echoConfig[ii].emergency = bufHead->echo.emergency;
      }
    }
    return RET_SUCCESS;
}

/***## IMMEDIATE EXECUTION COMMANDS #############################################################################################################################################***/

void commandStatus()
{
  paramType *tempBufTail;
  unsigned int queue = 0;
  int ii;

  // Look through all commands in the circular queue and add time from each of them
  tempBufTail = bufTail;
  while( tempBufTail != bufHead )
  {
    queue++;
    tempBufTail++;
    if( tempBufTail == bufCommand + COMMAND_BUF_LENGTH ) { tempBufTail = bufCommand; }
  }

  Serial.print(KeyREADY); Serial.print(KeyDELIMITER); Serial.print(motionInProgress); Serial.print(KeyDELIMITER); Serial.print(queue); Serial.print(KeyDELIMITER); Serial.print(COMMAND_BUF_LENGTH-1);
  Serial.print(KeyEOL1);

  Serial.print(KeyDRIVE); Serial.print(KeyDELIMITER); Serial.print(lastCommand.time);
  for( int ii=0; ii<4; ii++ ) { Serial.print(KeyDELIMITER); Serial.print(lastCommand.motor[ii]); }
  Serial.print(KeyEOL1);
  
  Serial.print(KeyMOVE); Serial.print(KeyDELIMITER); Serial.print(lastMove.distance); Serial.print(KeyDELIMITER); Serial.print(lastMove.velocity); Serial.print(KeyDELIMITER); Serial.print(lastMove.course); Serial.print(KeyDELIMITER); Serial.print(lastMove.curve);
  Serial.print(KeyEOL1);

  Serial.print(KeyMODE); Serial.print(KeyDELIMITER); Serial.print(currentMode); Serial.print(KeyDELIMITER); Serial.print(echoRepeat); Serial.print(KeyDELIMITER); Serial.print(echoNextSensor);
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);

  for( ii=0; ii<ECHO_SENSORS; ii++ ) {
    Serial.print(KeyECHO); Serial.print(KeyDELIMITER); Serial.print(ii); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].low); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].high);
    Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].emergency); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].sensors);
    Serial.print(KeyEOL1);
  }
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);
}

void commandHello() {
  char line_buf[ MAX_STRING_LENGTH + 1 ];
  strcpy_P(line_buf, (char*)pgm_read_word(&(string_table_local[0])));
  Serial.print(KeyHELLO); Serial.print(KeyDELIMITER); Serial.println(line_buf);
}

/***## MOTION CONTROL ######################################################################################################################################################***/

/****************************************************/
/* Convert command to motor and time values         */
/****************************************************/
void prepareDrive() {
    moveType move;
    if( DEBUG ) {
        Serial.print( "queue: " ); Serial.print( bufTail->params[0] ); Serial.print( ' ' ); Serial.print( bufTail->params[1] ); Serial.print( ' ' ); Serial.print( bufTail->params[2] );
        Serial.print( ' ' ); Serial.print( bufTail->params[3] ); Serial.print( ' ' ); Serial.print( bufTail->params[4] ); Serial.print( ' ' ); Serial.println( bufTail->command );
    }

    infiniteDelta = {0,0,0,0,0};

    switch( bufTail->command ) {
      case COMMAND_DRIVE:
        lastCommand = bufTail->drive;
        commandDrive();
        break;
      case COMMAND_MOVE:
        processMoveParameters(&(bufTail->move));
        commandDrive();
        lastMove = bufTail->move;
        break;
      case COMMAND_DELTA:
        move = bufTail->move;
        processDeltaParameters(&move);
        processMoveParameters(&move);
        commandDrive();
        lastMove = move;
        if( (bufTail->delta.repeat) == INFINITE_COMMAND ) {
            infiniteDelta = bufTail->delta;
        } else {
            if( bufTail->delta.repeat > 0 ) {
                (bufTail->delta.repeat)--;
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

    int local_power = bufTail->rotate.power;
    if ( bufTail->rotate.angle < 0 ) local_power = -local_power;
    
    float fixed_power = calibration(local_power, calRotation);
    for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = fixed_power; }
    
    if( (local_power == 0) || (abs(bufTail->rotate.angle) == INFINITE_COMMAND) ) {
        lastCommand.time = 0;
    } else {
        //TODO time can exceed MAX possible value. Need to check at validate stage
        lastCommand.time = CAR_CM_PER_DEG * CONST_MS_PER_SEC * CALIBRATION_ROTATE_POWER_CM_S * abs(bufTail->rotate.angle) / abs(local_power); // 1000 * 0.168 * 2 * angle /power
        if( lastCommand.time == 0 ) {
            for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
        }
    }

    if( DEBUG ) {
        Serial.print( "Rotate: " ); Serial.print( bufTail->rotate.angle ); Serial.print( ' ' ); Serial.println( bufTail->rotate.power );
        Serial.print( "Rotate: " ); Serial.print( lastCommand.time ); Serial.print( ' ' ); Serial.println( lastCommand.motor[0] );
    }
}

void processMoveParameters( moveType* mv ) {
  int angle_0_45, local_power, max_power, fixed_velocity;
  float course, rotation, correction, drive1, drive2;

  if( mv->distance == KEEP_PARAMETER ) { mv->distance = lastMove.distance; }
  if( mv->velocity == KEEP_PARAMETER ) { mv->velocity = lastMove.velocity; }
  if( mv->course == KEEP_PARAMETER ) { mv->course = lastMove.course; }
  if( mv->curve == KEEP_PARAMETER ) { mv->curve = lastMove.curve; }

  if( abs(mv->velocity) < MIN_POWER_MOVE / CALIBRATION_MOVE_POWER_CM_S ) {
    mv->velocity = 0;
    statusDecode(RET_WARN_MIN_POWER);
  }

  angle_0_45 = abs(mv->course) % 90;
  if(angle_0_45 > 45) angle_0_45 = 90 - angle_0_45;
  course = mv->course * CONST_PI / CONST_DEG_PER_PI;
  
  for( int jj=0; jj<10; jj++ ) {
    //TODO process error if jj=10 is not enough and power still more than MAX
    local_power = mv->velocity * CALIBRATION_MOVE_POWER_CM_S;
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
      fixed_velocity = mv->velocity * (float)MAX_COMMAND_POWER / max_power;
      if(fixed_velocity == mv->velocity) {
        if(fixed_velocity > 0) { (fixed_velocity)--; }
        if(fixed_velocity < 0) { (fixed_velocity)++; }
      }
      mv->velocity = fixed_velocity;
      if( DEBUG ) {
        Serial.print( "Motor: " ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
        Serial.print( "Power Reduced: " ); Serial.print( ' ' ); Serial.print( max_power ); Serial.print( ' ' ); Serial.print( mv->velocity ); Serial.print( ' ' ); Serial.println( local_power );
      }
    } else {
      if( jj > 1 ) { statusDecode(RET_WARN_MAX_POWER_REACHED); }
      break;
    }
  }

  if( (mv->velocity == 0) || (abs(mv->distance) == INFINITE_COMMAND) )
  {
      lastCommand.time = 0;
  } else {
      //TODO time can exceed MAX possible value. Need to check at validate stage
      lastCommand.time = CONST_MS_PER_SEC * (long)abs(mv->distance) / abs(mv->velocity); // 1000 * distance / velocity
      if( lastCommand.time == 0 ) {
          for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
      }
  }
  
  if( DEBUG ) {
    Serial.print( "Move Input: " ); Serial.print( mv->distance ); Serial.print( ' ' ); Serial.print( mv->velocity ); Serial.print( ' ' ); Serial.print( mv->course ); Serial.print( ' ' ); Serial.println( mv->curve );
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
        if (dm->distance > MAX_MOVE_DISTANCE) { dm->distance = MAX_MOVE_DISTANCE; }
        if (dm->distance < -MAX_MOVE_DISTANCE) { dm->distance = -MAX_MOVE_DISTANCE; }
    }
    
    dm->velocity = lastMove.velocity + dm->velocity;
    if (dm->velocity > MAX_MOVE_VELOCITY) { dm->velocity = MAX_MOVE_VELOCITY; }
    if (dm->velocity < -MAX_MOVE_VELOCITY) { dm->velocity = -MAX_MOVE_VELOCITY; }

    dm->course = lastMove.course + dm->course;
    dm->course = dm->course % MAX_MOVE_COURSE;

    dm->curve = lastMove.curve + dm->curve;
    if( dm->curve > MAX_MOVE_CURVE ) { dm->curve = MAX_MOVE_CURVE; }
    if( dm->curve < -MAX_MOVE_CURVE ) { dm->curve = -MAX_MOVE_CURVE; }
}

float calibration(float power, const calibrationType cal) {
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

/****************************************************/
/* Motor commands                                   */
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
      motionInProgress = 1;
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
  motionInProgress = 0;
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

/***## ECHO CONTROL ######################################################################################################################################################***/

void prepareEcho(void) {
    int ii, rotate = 0;
    if( isrEchoIndex != ECHO_ISR_COMPLETE ) { completeEcho(echoSensor); }
    for( ii=0; ii<ECHO_SENSORS; ii++ ) {
      if( ++echoSensor == ECHO_SENSORS ) { echoSensor = 0; rotate = 1; }
      if( currentMode & (0x0001 << echoSensor) ) {
        commandEcho(echoSensor);
        if( rotate ) { echoRepeatTime = millis() + echoRepeat; }
        else { echoRepeatTime = millis() + echoNextSensor; }
        break;
      }
    }  
}

void commandEcho(int num) {
  switch( PCISR_BIT(soundEchoPin[num]) ) { // Specify pin by mask and enable ISRs for corresponded pin
        case 0: PCMSK0 |= PCMSK_MASK(soundEchoPin[num]); break;
        case 1: PCMSK1 |= PCMSK_MASK(soundEchoPin[num]); break;
        case 2: PCMSK2 |= PCMSK_MASK(soundEchoPin[num]); break;
  }
  PCIFR  |= bit(PCISR_BIT(soundEchoPin[num])); // clear any outstanding interrupts
  PCICR  |= bit(PCISR_BIT(soundEchoPin[num])); // enable pins change interrupts for corresponded group
  isrEchoIndex = ECHO_ISR_START; // Unblock the ISR logic

  // Trigger ultrasonic sensor
  digitalWrite(soundTriggerPin[num], HIGH);
  delayMicroseconds(10);
  digitalWrite(soundTriggerPin[num], LOW);
}

void completeEcho(int num) {
    int report = 0;
    unsigned long range;
    PCICR  &= ~bit(PCISR_BIT(soundEchoPin[num])); // disable pins change interrupts for corresponded group
    switch( PCISR_BIT(soundEchoPin[num]) ) { // Specify pin by mask and disable ISRs for corresponded pin
          case 0: PCMSK0 &= ~PCMSK_MASK(soundEchoPin[num]); break;
          case 1: PCMSK1 &= ~PCMSK_MASK(soundEchoPin[num]); break;
          case 2: PCMSK2 &= ~PCMSK_MASK(soundEchoPin[num]); break;
    }
    PCIFR  |= bit(PCISR_BIT(soundEchoPin[num])); // clear any outstanding interrupts

    if( isrEchoIndex == ECHO_ISR_LAST ) {
        range = isrEchoTime[1] - isrEchoTime[0];
        if( range < (long)MAX_ECHO_RANGE_CM * SOUND_MICROS_PER_CM ) {
            echoConfig[num].sensors = range / SOUND_MICROS_PER_CM;
      
            if( motionInProgress && (echoConfig[num].sensors < echoConfig[num].emergency) ) {
                commandStop();
                statusDecode( RET_WARN_EMERGENCY_STOP );
            }
      
        } else {
            echoConfig[num].sensors = MAX_ECHO_RANGE_CM;
        }
    } else {
        echoConfig[num].sensors = MAX_ECHO_RANGE_CM;
    }
    isrEchoIndex = ECHO_ISR_COMPLETE; // Confirm we completed reading echo range      

    if( DEBUG ) { Serial.print(isrEchoTime[0]); Serial.print(KeyDELIMITER); Serial.print(isrEchoTime[1]); Serial.print(KeyDELIMITER); Serial.print(range); Serial.print(KeyDELIMITER); Serial.println(echoConfig[num].sensors); }

    if( echoConfig[num].low < echoConfig[num].high ) {
        if( (echoConfig[num].sensors >= echoConfig[num].low) && (echoConfig[num].sensors <= echoConfig[num].high) ) { report = 1; }
    }
    else if( echoConfig[num].low > echoConfig[num].high ) {
        if( (echoConfig[num].sensors >= echoConfig[num].low) || (echoConfig[num].sensors <= echoConfig[num].high) ) { report = 1; }
    }
    if( report ) {
        Serial.print(KeyRANGE); Serial.print(KeyDELIMITER); Serial.print(num); Serial.print(KeyDELIMITER); Serial.print(echoConfig[num].sensors);
        Serial.print(KeyEOL2); Serial.print(KeyEOL3);
    }
}

ISR (PCINT0_vect)
{
    if( isrEchoIndex < ECHO_ISR_LAST ) { isrEchoTime[isrEchoIndex++] = micros(); }
}

ISR (PCINT1_vect)
{
    if( isrEchoIndex < ECHO_ISR_LAST ) { isrEchoTime[isrEchoIndex++] = micros(); }
}

ISR (PCINT2_vect)
{
    if( isrEchoIndex < ECHO_ISR_LAST ) { isrEchoTime[isrEchoIndex++] = micros(); }
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
  if( millis() > echoRepeatTime ) {
    prepareEcho();
  }
  if( isrEchoIndex == ECHO_ISR_LAST ) {
    completeEcho(echoSensor);
  }
  if( timer == 0 )
  {
      if( bufTail != bufHead ) { prepareDrive(); }
      else { processInfiniteDelta(); }
  }
  else
  {
      if( millis() - setTime > timer )
      {
          if( bufTail != bufHead ) {
              prepareDrive();
          } else {
              if( processInfiniteDelta() != RET_SUCCESS ) { completeDrive(); }
          }
      }
  }
}

void serialEvent() {
  char tempBuf[MAX_STREAM_LENGTH / 2];
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
        digitalWrite(sharedTestResetPin, LOW);  // reset the board
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

