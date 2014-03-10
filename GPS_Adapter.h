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

#define GPS_NUMBAUDRATES (sizeof(gpsBaudRates)/sizeof(gpsBaudRates[0]))
#define GPS_NUMTYPES     (sizeof(gpsTypes)/sizeof(gpsTypes[0]))

// Timeout for GPS
#define GPS_MAXIDLE_DETECTING 200 // 2 seconds at 100Hz
#define GPS_MAXIDLE 500           // 5 seconds at 100Hz

extern struct gpsData gpsData;

struct gpsType {
  const char *name;
  void (*init)();
  int  (*processData)(unsigned char);
};

const unsigned long gpsBaudRates[] = { 
  9600L, 19200L, 38400L, 57600L, 115200L};
const struct gpsType gpsTypes[] = {
  { 
    "UBlox", ubloxInit, ubloxProcessData   }
  ,
};

void initializeGpsData();
void gpsSendConfig();
void initializeGps();
void updateGps();
bool haveAGpsLock();
long getCourse();
unsigned long getGpsSpeed();
unsigned long getGpsFixTime();
unsigned long getGpsAltitude();
void setProjectionLocation(struct GeodeticPosition pos);
void computeDistanceAndBearing(struct GeodeticPosition p1, struct GeodeticPosition p2);
float getDistanceMeter();
float getDistanceFoot();

#endif

