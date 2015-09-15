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
    RET_ERR_UNKNOWN,
    RET_ERR_MAX_NUMBER,
} Ret_Status;

/****************************************************/
/* Command data structures                          */
/****************************************************/
typedef struct moveType {
    int distance;
    int power;
    int course;
    int curve;
} moveType;

typedef struct deltaType {
    int distance;
    int power;
    int course;
    int curve;
    int repeat;
} deltaType;

typedef moveType lastEvType;

typedef struct driveType {
    int time;
    int motor[4];
} driveType;

typedef driveType driveEvType;

typedef struct modeType {
    int set;
    int reset;
} modeType;

typedef struct modeEvType {
    int mode;
    int queue;
} modeEvType;

typedef struct rotateType {
    int angle;
    int power;
} rotateType;

typedef struct echoType {
    int repeat;
} echoType;

typedef struct echoEvType {
    int range;
} echoEvType;

typedef struct readyEvType {
    int queue;
} readyEvType;


typedef union paramType {
  struct{
    int params[5];
    unsigned char command;
  };
  moveType m;
  deltaType md;
  lastEvType le;
  driveType d;
  driveEvType de;
  modeType o;
  modeEvType oe;
  rotateType r;
  echoType e;
  echoEvType ee;
  readyEvType re;
} paramType;

/****************************************************/
/* Mode                                             */
/****************************************************/
typedef enum modeEnum
{
  MODE_MOTOR_DISABLE = 0x0001,
  MODE_FW_UPGRADE_ENABLE = 0x0002,
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
static const char KeyLAST[] = "LAST";

/****************************************************/
/* Vehicle limits                                   */
/****************************************************/
static const int MAX_COMMAND_TIME = 30000;
static const int MAX_COMMAND_POWER = 255;
static const int MAX_WAY_CURVE = 100; // 1000/cm
static const int MAX_COMMAND_DISTANCE = 500; // cm; MAX_COMMAND_DISTANCE shall be less than INFINITE_COMMAND/2
static const int INFINITE_COMMAND = 30000;
static const int MAX_COMMAND_COURSE = 360; // degrees
static const int MAX_ECHO_RANGE_CM = 500;
static const int MIN_ECHO_REPEAT = 50; // mS

/****************************************************/
/* String table                                     */
/****************************************************/
#define STRING_WARN1 "Calibration exceeded max power. Replaced by possible value"
#define STRING_WARN2 "Too low power. Zero power used instead"
#define STRING_WARN3 "Unknown Warning"
#define STRING_ERROR1 "Buffer Overload"
#define STRING_ERROR2 "No Terminator"
#define STRING_ERROR3 "Wrong Command"
#define STRING_ERROR4 "No Parameters"
#define STRING_ERROR5 "No Delimiter"
#define STRING_ERROR6 "Garbage in Stream Buffer"
#define STRING_ERROR7 "Wrong Time Value"
#define STRING_ERROR8 "Wrong Power Value"
#define STRING_ERROR9 "Wrong Angle Value"
#define STRING_ERROR10 "Too large degree of curve"
#define STRING_ERROR11 "Wrong Course Value"
#define STRING_ERROR12 "Wrong Distance Value"
#define STRING_ERROR13 "Too small echo repeat time"
#define STRING_ERROR14 "Too large parameter value"
#define STRING_ERROR15 "Negative Repeat Value"
#define STRING_ERROR16 "Critical System Error"
#define STRING_ERROR17 "Unknown Error"

#define MAX_STRING_LENGTH 63

extern const char* const string_table_warn[];
extern const char* const string_table_error[];


