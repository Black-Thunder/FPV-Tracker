#ifndef GPS_TRACKER_H
#define GPS_TRACKER_H

#include "HMC5883L.h"

#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG 57.295779513082320876798154814105
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern HMC5883L compass;

extern bool isHomePositionSet;

void updateGCSPosition();
void updateGCSHeading();
void servoPathfinder(int angle_b, int angle_a);
void calculateTrackingVariables(float lon1, float lat1, float lon2, float lat2, int alt);
int calculateBearing(float lon1, float lat1, float lon2, float lat2);
int calculateElevation(float lon1, float lat1, float lon2, float lat2, int alt);
float toRad(float angle);
float toDeg(float angle);

#endif