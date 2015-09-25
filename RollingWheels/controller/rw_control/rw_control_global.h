#ifndef RW_CONTROL_GLOBAL_H
#define RW_CONTROL_GLOBAL_H

/****************************************************/
/* Error codes                                      */
/****************************************************/
typedef enum Ret_Status {
    RET_SUCCESS = 0,
    RET_NO_COMMAND,
    RET_NO_INFINITE_DELTA,
    RET_WARN_STATUS_START = 10,
    RET_WARN_MAX_POWER_REACHED,
    RET_WARN_MIN_POWER,
    RET_WARN_EMERGENCY_STOP,
    RET_WARN_UNKNOWN,
    RET_ERR_STATUS_START = 20,
    RET_ERR_QUEUE_OVERLOAD,
    RET_ERR_PARCE_TERMINATOR,
    RET_ERR_PARCE_COMMAND,
    RET_ERR_PARAM_NOT_FOUND,
    RET_ERR_PARAM_DELIMETER,
    RET_ERR_GARBAGE,
    RET_ERR_PARAM_VALUE_TIME,
    RET_ERR_PARAM_VALUE_POWER,
    RET_ERR_PARAM_VALUE_ANGLE,
    RET_ERR_PARAM_VALUE_CURVE,
    RET_ERR_PARAM_VALUE_COURSE,
    RET_ERR_PARAM_VALUE_DISTANCE,
    RET_ERR_ECHO_REPEAT,
    RET_ERR_PARAM_VALUE_MAX,
    RET_ERR_PARAM_VALUE_REPEAT,
    RET_ERR_SYSTEM_CRITICAL,
    RET_ERR_ECHO_DISTANCE,
    RET_ERR_UNKNOWN,
    RET_ERR_MAX_NUMBER,
} Ret_Status;

/****************************************************/
/* Command data structures                          */
/****************************************************/
typedef struct moveType {
    int distance;
    int velocity;
    int course;
    int curve;
} moveType;

typedef struct deltaType {
    int distance;
    int velocity;
    int course;
    int curve;
    int repeat;
} deltaType;

typedef moveType moveEvType;

typedef struct driveType {
    int time;
    int motor[4];
} driveType;

typedef driveType driveEvType;

typedef struct modeType {
    int set;
    int reset;
    int echoRepeat;
    int echoNextSensor;
} modeType;

typedef struct modeEvType {
    int mode;
    int echoRepeat;
    int echoNextSensor;
} modeEvType;

typedef struct rotateType {
    int angle;
    int power;
} rotateType;

typedef struct echoType {
    int sensors;
    int low;
    int high;
    int emergency;
} echoType;

typedef struct echoEvType {
    int sensor;
    int low;
    int high;
    int emergency;
    int range;
} echoEvType;

typedef struct rangeEvType {
    int sensor;
    int range;
} rangeEvType;

typedef struct readyEvType {
    int motion;
    int queue;
    int queueMax;
} readyEvType;


typedef union paramType {
  struct{
    int params[5];
    unsigned char command;
  };
  moveType move;
  deltaType delta;
  moveEvType moveEv;
  driveType drive;
  driveEvType driveEv;
  modeType mode;
  modeEvType modeEv;
  rotateType rotate;
  echoType echo;
  echoEvType echoEv;
  readyEvType readyEv;
} paramType;

/****************************************************/
/* Mode                                             */
/****************************************************/
typedef enum modeEnum
{
  MODE_ECHO_0 = 0x0001,
  MODE_ECHO_1 = 0x0002,
  MODE_ECHO_2 = 0x0004,
  MODE_ECHO_3 = 0x0008,
  MODE_MOTOR_DISABLE = 0x0010,
  MODE_FW_UPGRADE_ENABLE = 0x0020,
} modeEnum;

/****************************************************/
/* Parcer constants                                 */
/****************************************************/
static const char KeyDRIVE[] = "DRIVE";
static const char KeyMOVE[] = "MOVE";
static const char KeyDELTA[] = "DELTA";
static const char KeyROTATE[] = "ROTATE";
static const char KeySTOP[] = "STOP";
static const char KeySTATUS[] = "STATUS";
static const char KeyHELLO[] = "HELLO";
static const char KeyECHO[] = "ECHO";
static const char KeyMODE[] = "MODE";
static const char KeyEMPTY[] = "";
static const char KeyDELIMITER = ',';
static const char KeyEOL1 = ';';
static const char KeyEOL2 = 0x0D; // CR
static const char KeyEOL3 = 0x0A; // LF
static const char KeyREADY[] = "READY";
static const char KeyERROR[] = "ERROR";
static const char KeyWARN[] = "WARN";
static const char KeyRANGE[] = "RANGE";

/****************************************************/
/* Vehicle limits                                   */
/****************************************************/
static const int MAX_COMMAND_TIME = 30000;
static const int MAX_COMMAND_POWER = 255;
static const int MAX_MOVE_VELOCITY = 100; // cm/S
static const int MAX_MOVE_CURVE = 100; // 1000/cm
static const int MAX_MOVE_DISTANCE = 1000; // cm; MAX_COMMAND_DISTANCE shall be less than INFINITE_COMMAND/2; MAX_COMMAND_DISTANCE shall be less than MAX_INT * MAX_MOVE_VELOCITY / CONST_MS_PER_SEC
static const int INFINITE_COMMAND = 30000;
static const int MAX_MOVE_COURSE = 360; // degrees
static const int MAX_ECHO_RANGE_CM = 500;
static const int MIN_ECHO_REPEAT = 40; // mS
static const int ECHO_EMERGENCY_RANGE = 80; // cm

/****************************************************/
/* Vehicle geometry and Calibration constants       */
/****************************************************/
#define CONST_MS_PER_SEC (1000)
#define CONST_PI (3.1415)
#define CONST_DEG_PER_PI (180)
#define CAR_RADIUS (9.6)
static const float CAR_CM_PER_DEG = CAR_RADIUS * CONST_PI / CONST_DEG_PER_PI; //0.168
static const int SOUND_MICROS_PER_CM = 58; // doubled because the sound forward plus back way
static const int MIN_POWER_ROTATION = 50;
static const int MIN_POWER_MOVE = 70;
static const float CALIBRATION_ROTATE_POWER_CM_S = 2.; // Power per cm/S for ROTATE command
static const float CALIBRATION_MOVE_POWER_CM_S = 2.56; // Power per cm/S for MOVE command

/****************************************************/
/* String table                                     */
/****************************************************/
#define STRING_TABLE_GLOBAL \
const char String_Warn1[] STRING_MEM_MODE = "Calibration exceeded max power. Replaced by possible value"; \
const char String_Warn2[] STRING_MEM_MODE = "Too low power. Zero power used instead"; \
const char String_Warn3[] STRING_MEM_MODE = "Emergency stop"; \
const char String_Warn4[] STRING_MEM_MODE = "Unknown Warning"; \
const char String_Error1[] STRING_MEM_MODE = "Buffer Overload"; \
const char String_Error2[] STRING_MEM_MODE = "No Terminator"; \
const char String_Error3[] STRING_MEM_MODE = "Wrong Command"; \
const char String_Error4[] STRING_MEM_MODE = "No Parameters"; \
const char String_Error5[] STRING_MEM_MODE = "No Delimiter"; \
const char String_Error6[] STRING_MEM_MODE = "Garbage in Stream Buffer"; \
const char String_Error7[] STRING_MEM_MODE = "Wrong Time Value"; \
const char String_Error8[] STRING_MEM_MODE = "Wrong Power Value"; \
const char String_Error9[] STRING_MEM_MODE = "Wrong Angle Value"; \
const char String_Error10[] STRING_MEM_MODE = "Too large degree of curve"; \
const char String_Error11[] STRING_MEM_MODE = "Wrong Course Value"; \
const char String_Error12[] STRING_MEM_MODE = "Wrong Distance Value"; \
const char String_Error13[] STRING_MEM_MODE = "Too small echo repeat time"; \
const char String_Error14[] STRING_MEM_MODE = "Too large parameter value"; \
const char String_Error15[] STRING_MEM_MODE = "Negative Repeat Value"; \
const char String_Error16[] STRING_MEM_MODE = "Critical System Error"; \
const char String_Error17[] STRING_MEM_MODE = "Wrong Echo Distance Range"; \
const char String_Error18[] STRING_MEM_MODE = "Unknown Error"; \
\
const char* const string_table_warn[] STRING_MEM_MODE = {String_Warn1, String_Warn2, String_Warn3}; \
const char* const string_table_error[] STRING_MEM_MODE = {String_Error1, String_Error2, String_Error3, String_Error4, String_Error5, String_Error6, String_Error7, String_Error8, \
                                                  String_Error9, String_Error10, String_Error11, String_Error12, String_Error13, String_Error14, String_Error15, String_Error16, String_Error17};

#define MAX_STRING_LENGTH 63

extern const char* const string_table_warn[];
extern const char* const string_table_error[];

#endif

