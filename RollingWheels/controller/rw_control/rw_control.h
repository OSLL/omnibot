/****************************************************/
/* String table (local strings)                     */
/****************************************************/
#define STRING_HELLO "Rolling Wheels Ard. ver:0.026.1"

/****************************************************/
/* Calibration                                      */
/****************************************************/
typedef struct calibrationType_t {
  unsigned char cutoff;
  unsigned char turn;
  unsigned char shift;
  unsigned char min;
  float factor;
} calibrationType;

static calibrationType const calCurve = {35, 50, 0, 0, 1};
static calibrationType const calRotation = {30, 65, 13, 50, 2};
static calibrationType const calMove = {60, 200, 40, 70, 1};

/****************************************************/
/* Constants and vehicle geometry                   */
/****************************************************/
static const int CONST_MS_PER_SEC = 1000;
static const float CONST_PI = 3.1415;
static const int CONST_DEG_PER_PI = 180;
static const float CAR_RADIUS = 9.6; // cm
static const float ROTATION_MS_CM_PER_DEG = CAR_RADIUS * CONST_MS_PER_SEC * CONST_PI / CONST_DEG_PER_PI; //167; //CAR_RADIUS*2pi/360(degrees)*1000(mS)
static const int SOUND_MS_PER_CM = 58; // doubled because the sound forward plus back way

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

/****************************************************/
/* Limits                                           */
/****************************************************/
const int MAX_STREAM_LENGTH = 255;
const int COMMAND_BUF_LENGTH = 30;

/****************************************************/
/* Function declarations                            */
/****************************************************/
int readStream(char *buf, int len);
int detectBT(void);
Ret_Status parceCommand(char* const buf);
Ret_Status parceParameters ( char *head, int par_num );
Ret_Status queueCommand (void);
Ret_Status validateDriveParameters (void);
Ret_Status processMoveParameters (void);
Ret_Status processDeltaParameters (void);
Ret_Status processRotateParameters (void);
Ret_Status processModeParameters (void);
Ret_Status processEchoParameters (void);
float calibration(float power, calibrationType cal);
void statusDecode(Ret_Status ret);
void commandDrive(void);
void completeDrive(void);
void commandStop(void);
void commandStatus(void);
void commandHello(void);
void commandEcho(void);


