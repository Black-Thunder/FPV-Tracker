#ifndef GPS_TRACKER_H
#define GPS_TRACKER_H

#include "HMC5883L.h"
#include "TrackerGCS.h"
#include "GPS_Ublox.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG 57.295779513082320876798154814105
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// little wait to have a more precise fix of the current position since it's called after the first gps fix
#define MIN_NB_GPS_READ_TO_INIT_HOME 15  

extern HMC5883L compass;

extern bool isHomeBaseInitialized();

HMC5883L compass;

uint8_t countToInitHome = 0;

const float R = 6371000.0;    //in meters. Earth radius 6371km
float lonScaleDown = 0.0;

unsigned long previousFixTime = 0;

bool haveNewGpsPosition() {
	return (haveAGpsLock() && (previousFixTime != getGpsFixTime()));
}

void clearNewGpsPosition() {
	previousFixTime = getGpsFixTime();
}

bool isHomeBaseInitialized() {
	return homeLatitude != GPS_INVALID_ANGLE;
}

void calculateLongitudeScaling(int32_t lat) {
    float rads = (abs((float)lat) / 10000000.0) * 0.0174532925;
    lonScaleDown = cos(rads);
}

void updateGCSPosition() {
	if (haveNewGpsPosition()) {
		clearNewGpsPosition();

		if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
			countToInitHome++;
		}
		else {
			homeLatitude = gpsData.lat;
			homeLongitude = gpsData.lon;

                        calculateLongitudeScaling(homeLongitude);

			lcd.setCursor(0, 3);
			lcd.print("Home position OK    ");
		}
	}
}

void updateGCSHeading() {
	//Get the reading from the HMC5883L and calculate the heading
	MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.

        //Calibration values (hard-coded)
        scaled.XAxis *= 2.10;
        scaled.XAxis += -341.10;
        scaled.YAxis *= 1.10;
        scaled.YAxis += 93.87;
        float angle = atan2(scaled.YAxis, scaled. XAxis);
        
        if(angle < 0) angle += 2*PI;
        if(angle > 2*PI) angle -= 2*PI;
        
        homeBearing = (int)round(angle * 180/M_PI);
}

int16_t calculateBearing(int32_t lon1, int32_t lat1, int32_t lon2, int32_t lat2) {
    float dLat = (lat2 - lat1);
    float dLon = (float)(lon2 - lon1) * lonScaleDown;
    uavDistanceToHome = sqrt(sq(fabs(dLat)) + sq(fabs(dLon))) * 1.113195; // home dist in cm.
    int16_t b = (int)round( -90 + (atan2(dLat, -dLon) * 57.295775));
    if(b < 0) b += 360; 
    return b; 
}

int16_t calculateElevation(int32_t alt) {
    float at = atan2(alt, uavDistanceToHome);
    at = at * 57.2957795;
    int16_t e = (int16_t)round(at);
    return e;
}

void calculateTrackingVariables(int32_t lon1, int32_t lat1, int32_t lon2, int32_t lat2, int32_t alt) {
	//calculating Bearing & Elevation  in degree decimal
	trackingBearing = calculateBearing(lon1, lat1, lon2, lat2);
	trackingElevation = calculateElevation(alt);
}
#endif
