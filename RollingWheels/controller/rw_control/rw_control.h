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
    COMMAND_DELTA_INFINITE,
    COMMAND_MAX,
} commandEnum;

/****************************************************/
/* Calibration                                      */
/****************************************************/
static const int CALIBRATION_MAX_CYCLES = 10;
static const unsigned char CALIBRATION_ROTATE_SHIFT = 10;
static const unsigned char CALIBRATION_MOVE_SHIFT = 40;
static const float CALIBRATION_ROTATE_POWER_MM_S = 0.19; // Power per mm/s for ROTATE command
static const float CALIBRATION_MOVE_POWER_MM_S = 0.24; // Power per mm/s for MOVE command
static const unsigned char CALIBRATION_MOVE_ANGLE = 80;

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
void processMoveParameters ( moveType* mv );
void processDeltaParameters ( moveType* mv );
Ret_Status processModeParameters (void);
Ret_Status processEchoParameters (void);
Ret_Status processConfigParameters (void);
float calibration(float move, float rotation);
void statusDecode(Ret_Status ret);
void prepareDrive(void);
void commandDrive(void);
void completeDrive(void);
void commandStop( int motion );
void commandStatus(void);
void commandHello(void);
void prepareEcho(void);
void commandEcho(int num);
void completeEcho(int num);
void systemInit(void);


