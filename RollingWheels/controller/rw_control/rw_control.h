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

static calibrationType const calCurve = {35, 50, 0};
static calibrationType const calRotation = {30, 65, 13};
static calibrationType const calMove = {60, 200, 40};

/****************************************************/
/* Pins                                             */
/****************************************************/
static const unsigned int forceResetPin = 10;
static const unsigned char motorPin[4] = {2, 4, 7, 8 };
static const unsigned char motorPwmPin[4] = {3, 5, 6, 9 };
static const unsigned int soundPowerPin = 13;
//static const unsigned int soundGroundPin = A2;
static const unsigned int soundEchoPin = 12;
static const unsigned int soundTriggerPin = 11;
static const unsigned int testLoadPin = 10;

/****************************************************/
/* Limits                                           */
/****************************************************/
const int MAX_STREAM_LENGTH = 255;
const int COMMAND_BUF_LENGTH = 20;

/****************************************************/
/* ISR                                              */
/****************************************************/
const unsigned char ISR_ECHO_LAST = 2;
const unsigned char ISR_ECHO_OFF = 3;
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
void commandPrepare(void);
void statusDecode(Ret_Status ret);
void commandDrive(void);
void completeDrive(void);
void commandStop(void);
void commandStatus(void);
void commandHello(void);
void commandEcho(void);


