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

void updateGCSPosition() {
	if (haveNewGpsPosition()) {
		clearNewGpsPosition();

		if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
			countToInitHome++;
		}
		else {
			homeLatitude = gpsData.lat / 1.0e7f;
			homeLongitude = gpsData.lon / 1.0e7f;

			lcd.setCursor(0, 3);
			lcd.print("Home position OK    ");
		}
	}
}

void updateGCSHeading() {
	//Get the reading from the HMC5883L and calculate the heading
	MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.

	int angle = atan2(-scaled.YAxis, scaled.XAxis) / M_PI * 180; // angle is atan(-y/x)
	if (angle < 0) angle = angle + 360;
	homeBearing = angle;
}

float toRad(float angle) {
	// convert degrees to radians
	return angle*0.01745329; // (angle/180)*pi
}

float toDeg(float angle) {
	// convert radians to degrees.
	return angle*57.29577951;   // (angle*180)/pi
}

int calculateBearing(float lon1, float lat1, float lon2, float lat2) {
	// bearing calc, feeded in radian, output degrees
	float a;
	a = atan2(sin(lon2 - lon1)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2 - lon1));
	a = toDeg(a);

	if (a < 0) a = 360 + a;
	a = 360 - a;
	return (int)round(a);
}

int calculateElevation(float lon1, float lat1, float lon2, float lat2, int alt) {
	// feeded in radian, output in degrees
	float a, el, c, d, dLat, dLon;
	//calculating distance between uav & home	
	dLat = (lat2 - lat1);
	dLon = (lon2 - lon1);
	a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
	c = 2 * asin(sqrt(a));
	d = (R * c);
	uavDistanceToHome = d;
	el = atan((float)alt / (10 * d));// in radian
	el = toDeg(el); // in degree
	return (int)round(el);
}

void calculateTrackingVariables(float lon1, float lat1, float lon2, float lat2, int alt) {
	// (homelon, homelat, uavlon, uavlat, uavalt ) 
	// Return Bearing & Elevation angles in degree
	// converting to radian
	lon1 = toRad(lon1);
	lat1 = toRad(lat1);
	lon2 = toRad(lon2);
	lat2 = toRad(lat2);

	//calculating bearing in degree decimal
	trackingBearing = calculateBearing(lon1, lat1, lon2, lat2);

	//calculating distance between uav & home
	trackingElevation = calculateElevation(lon1, lat1, lon2, lat2, alt);
}
#endif
