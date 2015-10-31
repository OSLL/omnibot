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

const char String_Hello[] STRING_MEM_MODE = "Rolling Wheels Ard. ver:0.038";
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
unsigned int currentMode;
/****************************************************/
/* Motor time control                               */
/****************************************************/
unsigned long timer, setTime;
int motionStatus;
/****************************************************/
/* Calibration                                      */
/****************************************************/
float calibrationMove;
float calibrationRotate;
unsigned char calibrationAngle;
unsigned char calibrationShiftMove;
unsigned char calibrationShiftRotate;
/****************************************************/
/* Last command values                              */
/****************************************************/
driveType lastCommand;
moveType lastMove;
deltaType infiniteDelta;
/****************************************************/
/* Echo sound control                               */
/****************************************************/
echoType echoConfig[ECHO_SENSORS];
unsigned long echoRepeatTime;
int echoRepeat;
int echoNextSensor;
unsigned char echoSensor;
unsigned long volatile isrEchoTime[ECHO_ISR_LAST];
unsigned char volatile isrEchoIndex;

/***######################################################################################################################################################################***/

/****************************************************/
/* Initialization                                   */
/****************************************************/

void setup() {

  char test_buf[ MAX_STRING_LENGTH + 1 ];
  Ret_Status ret;
  int ii;
  
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
  
  // Initialize variables, engines command queue and stop engines
  commandStop( MOTION_STOP_INIT );
  commandHello();

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

void systemInit(void) {
  int ii;

  currentMode = 0;
  calibrationAngle = CALIBRATION_MOVE_ANGLE;
  calibrationMove = CALIBRATION_MOVE_POWER_MM_S;
  calibrationRotate = CALIBRATION_ROTATE_POWER_MM_S;
  calibrationShiftMove = CALIBRATION_MOVE_SHIFT;
  calibrationShiftRotate = CALIBRATION_ROTATE_SHIFT;
  lastMove = {0,0,0,0};
  infiniteDelta = {0,0,0,0,0};

  // Disable echo interrupts
  for( ii=0; ii<ECHO_SENSORS; ii++ ) {
    PCICR  &= ~bit(PCISR_BIT(soundEchoPin[ii])); // disable pins change interrupts for corresponded group
    switch( PCISR_BIT(soundEchoPin[ii]) ) { // Specify pin by mask and disable ISRs for corresponded pin
          case 0: PCMSK0 &= ~PCMSK_MASK(soundEchoPin[ii]); break;
          case 1: PCMSK1 &= ~PCMSK_MASK(soundEchoPin[ii]); break;
          case 2: PCMSK2 &= ~PCMSK_MASK(soundEchoPin[ii]); break;
    }
    PCIFR  |= bit(PCISR_BIT(soundEchoPin[ii])); // clear any outstanding interrupts
  }
  isrEchoIndex = ECHO_ISR_COMPLETE;
  
  // Initialize echo variables
  echoRepeat = ECHO_REPEAT_MIN;
  echoNextSensor = ECHO_REPEAT_MIN;
  echoSensor = 0;
  for( ii=0; ii<ECHO_SENSORS; ii++ ) {
    echoConfig[ii] = {ECHO_RANGE_CM_MAX,0,0,0};
  }
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
    if( bufHead->delta.repeat == PARAMETER_INFINITE ) {
      bufHead->command = COMMAND_DELTA_INFINITE;
    } else {
      bufHead->command = COMMAND_DELTA;
    }
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
  else if (! strncmp( buf, KeySTOP, strlen(KeySTOP) )) {
    if( (ret = parceParameters( buf + strlen(KeySTOP), sizeof(stopType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    commandStop( bufHead->stop.motion ); 
  }
  else if (! strncmp( buf, KeyCONFIG, strlen(KeyCONFIG) )) {
    if( (ret = parceParameters( buf + strlen(KeyCONFIG), sizeof(configType)/sizeof(int) )) != RET_SUCCESS ) return ret;
    if( (ret = processConfigParameters()) != RET_SUCCESS ) return ret;
  }
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
    if( motionStatus > MOTION_STOP ) return RET_ERR_MOTION_LOCKED;
    tempBufHead = bufHead + 1;
    if( tempBufHead == bufCommand + COMMAND_BUF_LENGTH ) { tempBufHead = bufCommand; }
    if( tempBufHead == bufTail) return RET_ERR_QUEUE_OVERLOAD;
    bufHead = tempBufHead;
    return RET_SUCCESS;
}

Ret_Status validateDriveParameters () {
  if( (bufHead->drive.time > COMMAND_TIME_MAX) || (bufHead->drive.time < 0)) return RET_ERR_PARAM_VALUE_TIME;
  for( int ii=0; ii<4; ii++ )
  {
    if( abs(bufHead->drive.motor[ii]) > COMMAND_POWER_MAX ) return RET_ERR_PARAM_VALUE_POWER;
  }
  return RET_SUCCESS;
}

Ret_Status validateMoveParameters () {
  if( (abs(bufHead->move.distance) > MOVE_DISTANCE_MAX) && (abs(bufHead->move.distance) != PARAMETER_INFINITE)
                                                        && (bufHead->move.distance != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_DISTANCE;
  if( (abs(bufHead->move.velocity) > MOVE_VELOCITY_MAX) && (bufHead->move.velocity != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_POWER;
  if( (abs(bufHead->move.course) > MOVE_COURSE_MAX) && (bufHead->move.course != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_COURSE;
  if( (abs(bufHead->move.curve) >= PARAMETER_INFINITE) && (bufHead->move.curve != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_CURVE;
  
  // Switch to rotate w/o movement if abs(curve) > MOVE_CURVE_MAX
  if( (bufHead->move.curve > MOVE_CURVE_MAX) && (bufHead->move.curve != PARAMETER_KEEP) ) bufHead->move.curve = MOVE_CURVE_MAX;
  if( bufHead->move.curve < -MOVE_CURVE_MAX ) bufHead->move.curve = -MOVE_CURVE_MAX;

  return RET_SUCCESS;
}

Ret_Status validateDeltaParameters () {
    if( (bufHead->delta.repeat < 0) || (bufHead->delta.repeat > PARAMETER_INFINITE) ) return RET_ERR_PARAM_VALUE_REPEAT;
    if( (abs(bufHead->delta.distance) > 2*MOVE_DISTANCE_MAX ) && (abs(bufHead->delta.distance) != PARAMETER_INFINITE)
                                                              && (bufHead->delta.distance != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_DISTANCE;
    if( (abs(bufHead->delta.distance) == PARAMETER_INFINITE) && (bufHead->delta.repeat > 1) ) return RET_ERR_PARAM_VALUE_DISTANCE;
    if( (abs(bufHead->delta.velocity) > 2*MOVE_VELOCITY_MAX) && (bufHead->delta.velocity != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_POWER;
    if( (abs(bufHead->delta.course) > MOVE_COURSE_MAX) && (bufHead->delta.course != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_COURSE;
    if( (abs(bufHead->delta.curve) > 2*MOVE_CURVE_MAX) && (bufHead->delta.curve != PARAMETER_KEEP) ) return RET_ERR_PARAM_VALUE_CURVE;

    if( bufHead->delta.repeat != PARAMETER_INFINITE ) {
        if( bufHead->delta.distance == PARAMETER_KEEP ) { bufHead->delta.distance = 0; }
        if( bufHead->delta.velocity == PARAMETER_KEEP ) { bufHead->delta.velocity = 0; }
        if( bufHead->delta.course == PARAMETER_KEEP ) { bufHead->delta.course = 0; }
        if( bufHead->delta.curve == PARAMETER_KEEP ) { bufHead->delta.curve = 0; }
    }
    return RET_SUCCESS;
}

Ret_Status processModeParameters () {
    unsigned int oldMode = currentMode;
    if( (bufHead->mode.echoRepeat != 0) && (bufHead->mode.echoRepeat < ECHO_REPEAT_MIN) ) return RET_ERR_ECHO_REPEAT;
    if( (bufHead->mode.echoNextSensor != 0) && (bufHead->mode.echoNextSensor < ECHO_REPEAT_MIN) ) return RET_ERR_ECHO_REPEAT;
    
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
    if( (bufHead->echo.low < 0) || (bufHead->echo.low > ECHO_RANGE_CM_MAX) ) return RET_ERR_ECHO_DISTANCE;
    if( (bufHead->echo.high < 0) || (bufHead->echo.high > ECHO_RANGE_CM_MAX) ) return RET_ERR_ECHO_DISTANCE;
    if( (bufHead->echo.emergency < 0) || (bufHead->echo.emergency > ECHO_RANGE_CM_MAX) ) return RET_ERR_ECHO_DISTANCE;
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

Ret_Status processConfigParameters () {
    calibrationMove = bufHead->config.move / 100.;
    calibrationRotate = bufHead->config.rotate / 100.;
    calibrationShiftMove = bufHead->config.shiftMove;
    calibrationShiftRotate = bufHead->config.shiftRotate;
    calibrationAngle = bufHead->config.angle;
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

  Serial.print(KeyREADY); Serial.print(KeyDELIMITER); Serial.print(motionStatus); Serial.print(KeyDELIMITER); Serial.print(queue); Serial.print(KeyDELIMITER); Serial.print(COMMAND_BUF_LENGTH-1);
  Serial.print(KeyEOL1);

  Serial.print(KeyDRIVE); Serial.print(KeyDELIMITER); Serial.print(lastCommand.time);
  for( int ii=0; ii<4; ii++ ) { Serial.print(KeyDELIMITER); Serial.print(lastCommand.motor[ii]); }
  Serial.print(KeyEOL1);
  
  Serial.print(KeyMOVE); Serial.print(KeyDELIMITER); Serial.print(lastMove.distance); Serial.print(KeyDELIMITER); Serial.print(lastMove.velocity);
  Serial.print(KeyDELIMITER); Serial.print(lastMove.course); Serial.print(KeyDELIMITER); Serial.print(lastMove.curve);
  Serial.print(KeyEOL1);

  Serial.print(KeyDELTA); Serial.print(KeyDELIMITER); Serial.print(infiniteDelta.distance); Serial.print(KeyDELIMITER); Serial.print(infiniteDelta.velocity);
  Serial.print(KeyDELIMITER); Serial.print(infiniteDelta.course); Serial.print(KeyDELIMITER); Serial.print(infiniteDelta.curve); Serial.print(KeyDELIMITER); Serial.print(infiniteDelta.repeat);
  Serial.print(KeyEOL1);

  Serial.print(KeyMODE); Serial.print(KeyDELIMITER); Serial.print(currentMode); Serial.print(KeyDELIMITER); Serial.print(echoRepeat); Serial.print(KeyDELIMITER); Serial.print(echoNextSensor);
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);

  for( ii=0; ii<ECHO_SENSORS; ii++ ) {
    Serial.print(KeyECHO); Serial.print(KeyDELIMITER); Serial.print(ii); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].low); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].high);
    Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].emergency); Serial.print(KeyDELIMITER); Serial.print(echoConfig[ii].sensors);
    Serial.print(KeyEOL1);
  }
  Serial.print(KeyEOL2); Serial.print(KeyEOL3);

  Serial.print(KeyCONFIG); Serial.print(KeyDELIMITER); Serial.print((int)(calibrationMove * 100)); Serial.print(KeyDELIMITER); Serial.print((int)(calibrationRotate * 100));
  Serial.print(KeyDELIMITER); Serial.print(calibrationShiftMove); Serial.print(KeyDELIMITER); Serial.print(calibrationShiftRotate); Serial.print(KeyDELIMITER); Serial.print(calibrationAngle);
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

    infiniteDelta.repeat = 0;

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
        if( bufTail->delta.repeat > 0 ) {
            move = bufTail->move;
            processDeltaParameters(&move);
            processMoveParameters(&move);
            commandDrive();
            lastMove = move;
            (bufTail->delta.repeat)--;
            if( bufTail->delta.repeat > 0 ) return; // Keep the delta command in the queue while repeate > 1
        }
        break;
      case COMMAND_DELTA_INFINITE:
        if( bufTail->delta.distance != PARAMETER_KEEP ) { infiniteDelta.distance = bufTail->delta.distance; }
        if( bufTail->delta.velocity != PARAMETER_KEEP ) { infiniteDelta.velocity = bufTail->delta.velocity; }
        if( bufTail->delta.course != PARAMETER_KEEP ) { infiniteDelta.course = bufTail->delta.course; }
        if( bufTail->delta.curve != PARAMETER_KEEP ) { infiniteDelta.curve = bufTail->delta.curve; }
        infiniteDelta.repeat = PARAMETER_INFINITE;
        processInfiniteDelta();
        break;
      default:
        statusDecode( RET_ERR_SYSTEM_CRITICAL );
        commandStop(MOTION_STOP_CRITICAL);
        return;
    }
    bufTail++;
    if( bufTail == bufCommand + COMMAND_BUF_LENGTH ) { bufTail = bufCommand; }
}

Ret_Status processInfiniteDelta() {
    deltaType delta;
    moveType* move;
    if( infiniteDelta.repeat == PARAMETER_INFINITE ) {
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

void processMoveParameters( moveType* mv ) {
  int angle_0_45, max_power, fixed_velocity;
  float course, rotation, linear_correction, linear_correction1, linear_correction2, rotate_correction, drive1, drive2;

  if( mv->distance == PARAMETER_KEEP ) { mv->distance = lastMove.distance; }
  if( mv->velocity == PARAMETER_KEEP ) { mv->velocity = lastMove.velocity; }
  if( mv->course == PARAMETER_KEEP ) { mv->course = lastMove.course; }
  if( mv->curve == PARAMETER_KEEP ) { mv->curve = lastMove.curve; }

  course = mv->course * CONST_PI / CONST_DEG_PER_PI;
  if( abs(mv->curve) != MOVE_CURVE_MAX ) {
    angle_0_45 = abs(mv->course) % 90;
    if(angle_0_45 > 45) { angle_0_45 = 90 - angle_0_45; }
    linear_correction = (1 + (float)angle_0_45/calibrationAngle) * calibrationMove;
    linear_correction1 = linear_correction * sin(course);
    linear_correction2 = linear_correction * cos(course);
  } else {
    linear_correction1 = 0;
    linear_correction2 = 0;
  }
  rotate_correction = CONST_CAR_RADIUS * mv->curve * calibrationRotate / 10000;  // CarRadius / WayRadius
  if ( mv->distance < 0 ) {
    linear_correction1 = -linear_correction1;
    linear_correction2 = -linear_correction2;
    rotate_correction = -rotate_correction;
  }
  
  for( int jj=0; jj<CALIBRATION_MAX_CYCLES; jj++ ) {
    if( jj == CALIBRATION_MAX_CYCLES-1 ) {
        mv->velocity = 0;
        statusDecode(RET_ERR_CALIBRATION_FAILED_POWER);
    }
    drive1 = mv->velocity * linear_correction1;
    drive2 = mv->velocity * linear_correction2;
    rotation = mv->velocity * rotate_correction;

    lastCommand.motor[0] = calibration(drive1, rotation);
    lastCommand.motor[1] = calibration(drive2, rotation);
    lastCommand.motor[2] = calibration(-drive1, rotation);
    lastCommand.motor[3] = calibration(-drive2, rotation);

    max_power = 0;
    for( int ii=0; ii<4; ii++ ) { max_power = max(max_power, abs(lastCommand.motor[ii])); }

    if( DEBUG ) {
      Serial.print( "Motor: " ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
      Serial.print( "Velocity: " ); Serial.print( ' ' ); Serial.print( max_power ); Serial.print( ' ' ); Serial.print( mv->velocity ); Serial.print( ' ' ); Serial.print( jj );
    }
    if( max_power < COMMAND_POWER_MIN ) {
      mv->velocity = 0;
      for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
      statusDecode(RET_WARN_MIN_POWER);
      break;
    }

    if( max_power > COMMAND_POWER_MAX ) {
      fixed_velocity = mv->velocity * (float)COMMAND_POWER_MAX / max_power;
      if(fixed_velocity == mv->velocity) {
        if(fixed_velocity > 0) { fixed_velocity -= 1/CALIBRATION_MOVE_POWER_MM_S; }
        if(fixed_velocity < 0) { fixed_velocity += 1/CALIBRATION_MOVE_POWER_MM_S; }
      }
      mv->velocity = fixed_velocity;
    } else {
      if( jj > 0 ) { statusDecode(RET_WARN_MAX_POWER_REACHED); }
      break;
    }
  }

  if( (mv->velocity == 0) || (abs(mv->distance) == PARAMETER_INFINITE) )
  {
      lastCommand.time = 0;
  } else {
      lastCommand.time = CONST_MS_PER_SEC * (long)abs(mv->distance) / abs(mv->velocity); // 1000 * distance / velocity
      if( lastCommand.time > COMMAND_TIME_MAX ) {
          lastCommand.time = 0;
          statusDecode(RET_ERR_CALIBRATION_FAILED_TIME);
      }
      if( lastCommand.time == 0 ) {
          for( int ii=0; ii<4; ii++ ) { lastCommand.motor[ii] = 0; }
      }
  }
  
  if( DEBUG ) {
    Serial.print( "Move Input: " ); Serial.print( mv->distance ); Serial.print( ' ' ); Serial.print( mv->velocity ); Serial.print( ' ' ); Serial.print( mv->course ); Serial.print( ' ' ); Serial.println( mv->curve );
    Serial.print( "Move1: " ); Serial.print( drive1 ); Serial.print( ' ' ); Serial.print( drive2 ); Serial.print( ' ' ); Serial.println( rotation );
    Serial.print( "Move2: " ); Serial.print( calibration(drive1, calibrationShiftMove) ); Serial.print( ' ' ); Serial.print( calibration(drive1+rotation, calibrationShiftRotate) ); Serial.print( ' ' ); Serial.println( calibration(drive1, calibrationShiftRotate) );
    Serial.print( "Move Output: " ); Serial.print( lastCommand.time ); Serial.print( ' ' ); Serial.print( lastCommand.motor[0] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[1] ); Serial.print( ' ' ); Serial.print( lastCommand.motor[2] ); Serial.print( ' ' ); Serial.println( lastCommand.motor[3] );
  }
}

void processDeltaParameters( moveType* dm ) {

    if( abs(dm->distance) != PARAMETER_INFINITE ) {
      if( abs(lastMove.distance) != PARAMETER_INFINITE ) { dm->distance = lastMove.distance + dm->distance; }
      else { dm->distance = lastMove.distance; }
    }
    if( abs(dm->distance) != PARAMETER_INFINITE ) {
        if (dm->distance > MOVE_DISTANCE_MAX) { dm->distance = MOVE_DISTANCE_MAX; }
        if (dm->distance < -MOVE_DISTANCE_MAX) { dm->distance = -MOVE_DISTANCE_MAX; }
    }
    
    dm->velocity = lastMove.velocity + dm->velocity;
    if (dm->velocity > MOVE_VELOCITY_MAX) { dm->velocity = MOVE_VELOCITY_MAX; }
    if (dm->velocity < -MOVE_VELOCITY_MAX) { dm->velocity = -MOVE_VELOCITY_MAX; }

    dm->course = lastMove.course + dm->course;
    dm->course = dm->course % MOVE_COURSE_MAX;

    dm->curve = lastMove.curve + dm->curve;
    if( dm->curve > MOVE_CURVE_MAX ) { dm->curve = MOVE_CURVE_MAX; }
    if( dm->curve < -MOVE_CURVE_MAX ) { dm->curve = -MOVE_CURVE_MAX; }
}

float calibration(float move, float rotation) {
  float power, sum, shift;

  move *= (1-calibrationShiftMove/(float)COMMAND_POWER_MAX);
  rotation *= (1-calibrationShiftRotate/(float)COMMAND_POWER_MAX);
  power = move + rotation;
  if ( abs(power) < 1 ) { return 0; } // Avoid rounding error.
  
  sum = abs(move) + abs(rotation);
  shift = calibrationShiftMove*abs(move)/sum + calibrationShiftRotate*abs(rotation)/sum;
  if( power > 0 ) { return (power + shift); }
  else { return (power - shift); }
}

/****************************************************/
/* Motor commands                                   */
/****************************************************/

void commandDrive()
{
    if( (lastCommand.time > COMMAND_TIME_MAX) || (lastCommand.time < 0)) {
      statusDecode( RET_ERR_PARAM_VALUE_TIME );
    } else {
      for( int ii=0; ii<4; ii++ )
      {
        if( ! (currentMode & MODE_MOTOR_DISABLE) ) {
            if( lastCommand.motor[ii] > 0 )
            {
              digitalWrite(motorPin[ii], HIGH);
              analogWrite(motorPwmPin[ii], COMMAND_POWER_MAX - lastCommand.motor[ii]);
            } else {
              digitalWrite(motorPin[ii], LOW);
              analogWrite(motorPwmPin[ii], -(lastCommand.motor[ii]));
            }
        }
      }
      timer = lastCommand.time;
      setTime = millis();
      motionStatus = MOTION_IN_PROGRESS;
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
  motionStatus = MOTION_STOP;
}

void commandStop( int motion )
{
    if( motion == MOTION_STOP_INIT ) { systemInit(); }
    // Purge commands from circular queue
    bufHead = bufCommand;
    bufTail = bufCommand;
    // Stop infinite delta
    infiniteDelta.repeat = 0;
    // Stop current command immediately
    completeDrive();
    if( motion > MOTION_STOP_INIT ) { motionStatus = motion; }
}

/***## ECHO CONTROL ######################################################################################################################################################***/

void prepareEcho(void) {
    int ii, rotate = 0;
    if( isrEchoIndex != ECHO_ISR_COMPLETE ) { completeEcho(echoSensor); }
    for( ii=0; ii<ECHO_SENSORS; ii++ ) {
      if( ++echoSensor == ECHO_SENSORS ) { echoSensor = 0; rotate = 1; }
      if( currentMode & (0x0001 << echoSensor) ) {
        commandEcho(echoSensor);
        if( rotate ) { echoRepeatTime += echoRepeat; }
        else { echoRepeatTime += echoNextSensor; }
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
        if( range < (long)ECHO_RANGE_CM_MAX * SOUND_MICROS_PER_CM ) {
            echoConfig[num].sensors = range / SOUND_MICROS_PER_CM;
      
            if( (motionStatus == MOTION_IN_PROGRESS) && (echoConfig[num].sensors < echoConfig[num].emergency) ) {
                commandStop( MOTION_STOP_LOCKED );
                statusDecode( RET_WARN_EMERGENCY_STOP );
            }
      
        } else {
            echoConfig[num].sensors = ECHO_RANGE_CM_MAX;
        }
    } else {
        echoConfig[num].sensors = ECHO_RANGE_CM_MAX;
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
  int bytes = 0;
  int symbol;
  while( bytes < len ) {
      symbol = Serial.read();
      if( symbol < 0 ) break;
      buf[bytes] = (char)symbol;
      bytes++;
  }
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

