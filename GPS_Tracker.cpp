#include "GPS_Tracker.h"
#include "TrackerGCS.h"
#include "GPS_Adapter.h"
#include <math.h>

HMC5883L compass;
bool isHomePositionSet = false;

const float R = 6371000.0;    //in meters. Earth radius 6371km

void updateGCSPosition() {
	if (haveAGpsLock()) {
		homeLatitude = gpsData.lat / 1.0e7f;
		homeLongitude = gpsData.lon / 1.0e7f;
		isHomePositionSet = true;
	}
}

void updateGCSHeading() {
	//Get the reading from the HMC5883L and calculate the heading
	MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.

	int angle = atan2(-scaled.YAxis , scaled.XAxis) / M_PI * 180; // angle is atan(-y/x)
	if(angle < 0) angle = angle  + 360;
	homeBearing = angle;
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

int calculateBearing(float lon1, float lat1, float lon2, float lat2) {
	// bearing calc, feeded in radian, output degrees
	float a;
	a = atan2(sin(lon2 - lon1)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2 - lon1));
	a = toDeg(a);

	if (a < 0) a = 360 + a;
	a = 360-a;
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

float toRad(float angle) {
	// convert degrees to radians
	return angle*0.01745329; // (angle/180)*pi
}

float toDeg(float angle) {
	// convert radians to degrees.
	return angle*57.29577951;   // (angle*180)/pi
}
