#ifndef GPS_TRACKER_H
#define GPS_TRACKER_H

#include "HMC5883L.h"

#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG 57.295779513082320876798154814105
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// little wait to have a more precise fix of the current position since it's called after the first gps fix
#define MIN_NB_GPS_READ_TO_INIT_HOME 15  

extern HMC5883L compass;

extern bool isHomeBaseInitialized();
bool haveNewGpsPosition();
void clearNewGpsPosition();
void updateGCSPosition();
void updateGCSHeading();
void calculateTrackingVariables(float lon1, float lat1, float lon2, float lat2, int alt);
int calculateBearing(float lon1, float lat1, float lon2, float lat2);
int calculateElevation(float lon1, float lat1, float lon2, float lat2, int alt);
float toRad(float angle);
float toDeg(float angle);

#endif
