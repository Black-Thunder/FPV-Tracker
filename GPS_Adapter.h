#ifndef GPSADAPTER_H
#define GPSADAPTER_H

#include "GPS_DataType.h"
#include "GPS_Ublox.h"
#include "HWSerial.h"
#include <stdint.h>

#define GPS_SERIAL Serial2
#define MIN_NB_SATS_IN_USE 6
#define GPS2RAD (1.0/572957795.0)
#define RAD2DEG 57.2957795

// Timeout for GPS
#define GPS_MAXIDLE_DETECTING 200 // 2 seconds at 100Hz
#define GPS_MAXIDLE 500           // 5 seconds at 100Hz

extern struct gpsData gpsData;

void initializeGpsData();
void initializeGps();
void updateGps();
bool haveAGpsLock();
long getCourse();
unsigned long getGpsSpeed();
unsigned long getGpsFixTime();
unsigned long getGpsAltitude();

#endif

