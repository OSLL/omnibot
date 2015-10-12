#include <time.h>
#include <sys/time.h>

/************************************************/
/* Funcion declarations                         */
/************************************************/

/************************************************/
/*                                              */
/************************************************/

void sleep_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000 ) * 1000000;
    nanosleep(&ts, NULL);
}

long long current_timestamp(void) {
    struct timeval te;
    gettimeofday(&te, NULL);
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    return milliseconds;
}

