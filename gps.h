#include <time.h>

/* Unsigned integer types  */
#define uint8_t  unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long

#ifndef _def_bool
#define _def_bool
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

typedef struct SituationData
{
    // Admin
    int         running;
    pthread_mutex_t lock;

    // From GPS.
    uint32_t    lastFixSinceMidnightUTC;
    float       Lat;
    float       Lng;
    uint8_t     quality;
    float       GeoidSep;               // geoid separation, ft, MSL minus HAE (used in altitude calculation)
    uint16_t    Satellites;             // satellites used in solution
    uint16_t    SatellitesTracked;      // satellites tracked (almanac data received)
    uint16_t    SatellitesSeen;         // satellites seen (signal received)
    float       Accuracy;               // 95% confidence for horizontal position, meters.
    float       Alt;                    // Feet MSL
    float       AccuracyVert;           // 95% confidence for vertical position, meters
    float       GPSVertVel;             // GPS vertical velocity, feet per second
    struct tm   LastFixLocalTime;
    uint16_t    TrueCourse;
    uint16_t    GroundSpeed;
    struct tm   LastGroundTrackTime;
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;

} SituationData;

void * run_gps(void *);
SituationData * gps_data(void);

