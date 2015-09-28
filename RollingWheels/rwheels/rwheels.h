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

/****************************************************/
/* Controller communication                         */
/****************************************************/
extern controllerDataType controllerData;
extern int seriald;

/************************************************/
/* Parcer API                                   */
/************************************************/
void printCurrentData(void);
void serialReadLine(void);

#endif
