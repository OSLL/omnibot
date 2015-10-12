#ifndef RWHEELS_H
#define RWHEELS_H

#define STRING_MEM_MODE

/****************************************************/
/* Data types                                       */
/****************************************************/
typedef struct controllerDataType {
    readyEvType ready;
    driveEvType drive;
    moveEvType move;
    modeEvType mode;
    echoEvType echo[ECHO_SENSORS];
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
extern controllerDataType controllerData;
extern int seriald;

/****************************************************/
/* Callbacks                                        */
/****************************************************/
typedef enum callbackEnum {
    CALLBACK_RANGE = 0,
    CALLBACK_EMERGENCY,
    CALLBACK_START,
    CALLBACK_TERMINATOR,
} callbackEnum;

typedef struct callbackRangeType {
    int sensor;
    int range;
} callbackRangeType;

typedef union callbackDataType {
    struct {
        int params[2];
    };
    callbackRangeType range;
} callbackDataType;

extern void (*callbacks[CALLBACK_TERMINATOR])(callbackDataType* data);

/************************************************/
/* RW API                                       */
/************************************************/
void rwstop(void);

/************************************************/
/* Parcer API                                   */
/************************************************/
void printCurrentData(void);
void serialReadLine(void);

/************************************************/
/* Library API                                  */
/************************************************/
void sleep_ms(int milliseconds);
long long current_timestamp(void);

#endif
