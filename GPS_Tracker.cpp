#include "GPS_Tracker.h"
#include "TrackerGCS.h"
#include "GPS_Adapter.h"
#include <math.h>

HMC5883L compass;
bool isHomePositionSet = false;

void updateGCSPosition() {
	if (haveAGpsLock()) {
		homeLatitude = gpsData.lat / 1.0e7f;
		homeLongitude = gpsData.lon / 1.0e7f;

		isHomePositionSet = true;
	}

	//TODO remove/debug
	Serial.print("State: "); Serial.print(gpsData.state);
	Serial.print(" Lat: "); Serial.print(gpsData.lat); Serial.print(" "); Serial.print(homeLatitude, 5);
	Serial.print(" Lon: "); Serial.print(gpsData.lon); Serial.print(" "); Serial.print(homeLongitude, 5);
	Serial.print(" Sats: "); Serial.print(gpsData.sats); Serial.println();
}

void updateGCSHeading() {
	//Get the reading from the HMC5883L and calculate the heading
	MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
	float heading = atan2(scaled.YAxis, scaled.XAxis);

	// Correct for when signs are reversed.
	if (heading < 0) heading += 2 * PI;
	if (heading > 2 * PI) heading -= 2 * PI;

	homeBearing = (int)heading * RAD_TO_DEG; //radians to degrees

	//TODO remove/debug
//	Serial.print(" Head: "); Serial.print(homeBearing); Serial.println();
}

void servoPathfinder(int angle_b, int angle_a){   // ( bearing, elevation )
	//find the best way to move pan servo considering 0� reference face toward
	if (angle_b <= 180) {
		if (horizontalMax >= angle_b) {
			if (angle_a <= verticalMin) {
				// checking if we reach the min tilt limit
				angle_a = verticalMin;
			}
			else if (angle_a > verticalMax) {
				//shouldn't happend but just in case
				angle_a = verticalMax;
			}
		}
		else if (horizontalMax < angle_b) {
			//relevant for 180� tilt config only, in case bearing is superior to pan_maxangle
			angle_b = 360 - (180 - angle_b);
			if (angle_b >= 360) {
				angle_b = angle_b - 360;
			}
			// invert pan axis 
			if (verticalMax >= (180 - angle_a)) {
				// invert pan & tilt for 180� Pan 180� Tilt config
				angle_a = 180 - angle_a;
			}
			else if (verticalMax < (180 - angle_a)) {
				// staying at nearest max pos
				angle_a = verticalMax;
			}
		}
	}
	else {
		if (horizontalMin > 360 - angle_b) {
			//works for all config
			if (angle_a < verticalMin) {
				// checking if we reach the min tilt limit
				angle_a = verticalMin;
			}
		}
		else if (horizontalMin <= 360 - angle_b) {
			angle_b = angle_b - 180;
			if (verticalMax >= (180 - angle_a)) {
				// invert pan & tilt for 180/180 conf
				angle_a = 180 - angle_a;
			}
			else if (verticalMax < (180 - angle_a)) {
				// staying at nearest max pos
				angle_a = verticalMax;
			}
		}
	}

	angle_a = constrain(angle_a, verticalMin, verticalMax);
	angle_b = constrain(angle_b, horizontalMin, horizontalMax);

	HorizontalServo.write(angle_b);
	VerticalServo.write(angle_a);
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
	return (int)round(a);
}

int calculateElevation(float lon1, float lat1, float lon2, float lat2, int alt) {
	// feeded in radian, output in degrees
	float a, el, c, d, R, dLat, dLon;
	//calculating distance between uav & home
	R = 6371000.0;    //in meters. Earth radius 6371km
	dLat = (lat2 - lat1);
	dLon = (lon2 - lon1);
	a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
	c = 2 * asin(sqrt(a));
	d = (R * c);
	uavDistanceToHome = d / 10;
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
