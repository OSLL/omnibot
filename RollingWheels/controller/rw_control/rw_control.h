/****************************************************/
/* String table memory allocation modifyer          */
/****************************************************/
#define STRING_MEM_MODE PROGMEM

/****************************************************/
/* Error codes                                      */
/****************************************************/
typedef enum commandEnum {
    COMMAND_DRIVE = 0,
    COMMAND_MOVE,
    COMMAND_DELTA,
    COMMAND_ROTATE,
    COMMAND_MAX,
} commandEnum;

/****************************************************/
/* Calibration                                      */
/****************************************************/
typedef struct calibrationType_t {
  /* memory allocation: 8 bytes per structure */
  unsigned char cutoff;
  unsigned char turn;
  unsigned char shift;
} calibrationType;

static calibrationType const calCurve = {20, 50, 0};
static calibrationType const calRotation = {30, 65, 13};
static calibrationType const calMove = {60, 200, 40};
static const int CALIBRATION_MAX_CYCLES = 10;
static const float CALIBRATION_ROTATE_POWER_CM_S = 2.; // Power per cm/S for ROTATE command
static const float CALIBRATION_MOVE_POWER_CM_S = 2.56; // Power per cm/S for MOVE command

/****************************************************/
/* Limits                                           */
/****************************************************/
static const int MAX_STREAM_LENGTH = 255;
static const int COMMAND_BUF_LENGTH = 20;

/****************************************************/
/* Pins                                             */
/****************************************************/
static const unsigned int sharedTestResetPin = 10;
static const unsigned char motorPin[4] = {2, 4, 7, 8 };
static const unsigned char motorPwmPin[4] = {3, 5, 6, 9 };
static const unsigned char soundEchoPin[ECHO_SENSORS] = {14, 17, 16, 15};
static const unsigned char soundTriggerPin[ECHO_SENSORS] = {11, 12, 19, 18};

/****************************************************/
/* ISR                                              */
/****************************************************/
typedef enum echoISREnum {
    ECHO_ISR_START = 0,
    ECHO_ISR_LAST = 2,
    ECHO_ISR_COMPLETE = 3,
} echoISREnum;

#define PCMSK_MASK(x) (bit( ((x)%14) %8))
#define PCISR_BIT(x) ( (( (((x) +8)%22) +8)%24) /8)

/****************************************************/
/* Function declarations                            */
/****************************************************/
int readStream(char *buf, int len);
int detectBT(void);
Ret_Status parceCommand(char* const buf);
Ret_Status parceParameters ( char *head, int par_num );
Ret_Status queueCommand (void);
Ret_Status validateDriveParameters (void);
Ret_Status validateMoveParameters (void);
Ret_Status validateDeltaParameters (void);
Ret_Status validateRotateParameters (void);
void processMoveParameters ( moveType* mv );
void processDeltaParameters ( moveType* mv );
void processRotateParameters (void);
Ret_Status processModeParameters (void);
Ret_Status processEchoParameters (void);
float calibration(float power, const calibrationType cal);
void statusDecode(Ret_Status ret);
void prepareDrive(void);
void commandDrive(void);
void completeDrive(void);
void commandStop(void);
void commandStatus(void);
void commandHello(void);
void prepareEcho(void);
void commandEcho(int num);
void completeEcho(int num);


