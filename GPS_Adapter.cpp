#include "GPS_Adapter.h"
#include "HWSerial.h"
#include <math.h>

GeodeticPosition currentPosition;

struct gpsData gpsData;

bool isGPSConfigured = false;

void initializeGpsData() {
	gpsData.lat = GPS_INVALID_ANGLE;
	gpsData.lon = GPS_INVALID_ANGLE;
	gpsData.course = GPS_INVALID_ANGLE;
	gpsData.speed = GPS_INVALID_SPEED;
	gpsData.height = GPS_INVALID_ALTITUDE;
	gpsData.accuracy = GPS_INVALID_ACCURACY;
	gpsData.fixage = GPS_INVALID_AGE;
	gpsData.state = GPS_DETECTING;
	gpsData.sentences = 0;
	gpsData.sats = 0;
	gpsData.fixtime = 0xFFFFFFFF;
}

// Initialize GPS subsystem (called once after powerup)
void initializeGps() {
	GPS_SERIAL.begin(9600);
	isGPSConfigured = ubloxInit();
	initializeGpsData();
}

// Read data from GPS, this should be called at 100Hz to make sure no data is lost
// due to overflowing serial input buffer
void updateGps() {
	gpsData.idlecount++;

	while (GPS_SERIAL.available()) {
		unsigned char c = GPS_SERIAL.read();
		int ret = 0;

		ret = ubloxProcessData(c);

		// Upon a successfully parsed sentence, zero the idlecounter and update position data
		if (ret) {
			if (gpsData.state == GPS_DETECTING) {
				gpsData.state = GPS_NOFIX; // make sure to lose detecting state (state may not have been updated by parser)
			}
			gpsData.idlecount = 0;
			currentPosition.latitude = gpsData.lat;
			currentPosition.longitude = gpsData.lon;
			currentPosition.altitude = gpsData.height;
		}
	}

	// Check for inactivity, we have two timeouts depending on scan status
	if (gpsData.idlecount > ((gpsData.state == GPS_DETECTING) ? GPS_MAXIDLE_DETECTING : GPS_MAXIDLE)) {
		gpsData.idlecount = 0;
		if (gpsData.state == GPS_DETECTING) {
			GPS_SERIAL.begin(9600);
		}

		// ensure detection state (if we lost connection to GPS)
		gpsData.state = GPS_DETECTING;

		ubloxInit();

		// zero GPS state
		initializeGpsData();
	}
}

bool haveAGpsLock() {
	return (gpsData.state > GPS_NOFIX) && (gpsData.sats >= MIN_NB_SATS_IN_USE);
}

long getCourse() {
	return gpsData.course / 1000; // to whole degrees
}
unsigned long getGpsSpeed() {
	return gpsData.speed;
}

unsigned long getGpsFixTime() {
	return gpsData.fixtime;
}

unsigned long getGpsAltitude() {
	return gpsData.height;
}
