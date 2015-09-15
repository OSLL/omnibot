/****************************************************/
/* Data types                                       */
/****************************************************/
typedef struct controllerDataType {
    readyEvType ready;
    driveEvType drive;
    lastEvType last;
    modeEvType mode;
    echoEvType echo;
} controllerDataType;

/****************************************************/
/* Constants                                        */
/****************************************************/
static const int DEBUG = 0;

/****************************************************/
/* Extern data                                      */
/****************************************************/
extern const char* const string_table_warn[];
extern const char* const string_table_error[];

/****************************************************/
/* Controller communication                         */
/****************************************************/
extern controllerDataType controllerData;
extern int seriald;
void serialReadLine( void );
