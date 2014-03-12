#include <stdint.h>
#include "GPS_Ublox.h"
#include "GPS_Adapter.h"

ubloxState ubloxProcessDataState;
ublox_message ubloxMessage;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;
unsigned short ubloxClass, ubloxId;
unsigned char  ubloxCKA, ubloxCKB;

// Initialize parser
void ubloxInit() {
	ubloxProcessDataState = WAIT_SYNC1;
}

// process complete binary packet
void ubloxParseData() {// uses publib vars
	gpsData.sentences++;
	if (ubloxClass == 1) { // NAV
		if (ubloxId == 2) { // NAV:POSLLH
			gpsData.lat = ubloxMessage.nav_posllh.lat;
			gpsData.lon = ubloxMessage.nav_posllh.lon;
			gpsData.height = ubloxMessage.nav_posllh.height;
			gpsData.accuracy = ubloxMessage.nav_posllh.hAcc;
			gpsData.fixtime = ubloxMessage.nav_posllh.iTow;
		}
		else if (ubloxId == 3) { //NAV:STATUS
			switch (ubloxMessage.nav_status.gpsFix) {
			case 2:
				gpsData.state = GPS_FIX2D;
				break;

			case 3:
				gpsData.state = GPS_FIX3D;
				break;

			default:
				gpsData.state = GPS_NOFIX;
				break;
			}
		}
		else if (ubloxId == 6) { // NAV:SOL
			gpsData.sats = ubloxMessage.nav_sol.numSV;
		}
		else if (ubloxId == 18) { // NAV:VELNED
			gpsData.course = ubloxMessage.nav_velned.heading / 100; // 10E-5 to millidegrees
			gpsData.speed = ubloxMessage.nav_velned.gSpeed;
			gpsData.velN = ubloxMessage.nav_velned.velN;
			gpsData.velE = ubloxMessage.nav_velned.velE;
			gpsData.velD = ubloxMessage.nav_velned.velD;
		}
	}
}

// process serial data
int ubloxProcessData(unsigned char data) {

	int parsed = 0;

	switch (ubloxProcessDataState) {
	case WAIT_SYNC1:
		if (data == 0xb5) {
			ubloxProcessDataState = WAIT_SYNC2;
		}
		break;

	case WAIT_SYNC2:
		if (data == 0x62) {
			ubloxProcessDataState = GET_CLASS;
		}
		else if (data == 0xb5) {
			// ubloxProcessDataState = GET_SYNC2;
		}
		else {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		break;
	case GET_CLASS:
		ubloxClass = data;
		ubloxCKA = data;
		ubloxCKB = data;
		ubloxProcessDataState = GET_ID;
		break;

	case GET_ID:
		ubloxId = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LL;
		break;

	case GET_LL:
		ubloxExpectedDataLength = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LH;
		break;

	case GET_LH:
		ubloxExpectedDataLength += data << 8;
		ubloxDataLength = 0;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		if (ubloxExpectedDataLength <= sizeof(ubloxMessage)) {
			ubloxProcessDataState = GET_DATA;
		}
		else {
			// discard overlong message
			ubloxProcessDataState = WAIT_SYNC1;
		}
		break;

	case GET_DATA:
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		// next will discard data if it exceeds our biggest known msg
		if (ubloxDataLength < sizeof(ubloxMessage)) {
			ubloxMessage.raw[ubloxDataLength++] = data;
		}
		if (ubloxDataLength >= ubloxExpectedDataLength) {
			ubloxProcessDataState = GET_CKA;
		}
		break;

	case GET_CKA:
		if (ubloxCKA != data) {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		else {
			ubloxProcessDataState = GET_CKB;
		}
		break;

	case GET_CKB:
		if (ubloxCKB == data) {
			parsed = 1;
			ubloxParseData();
		}
		ubloxProcessDataState = WAIT_SYNC1;
		break;

	}
	return parsed;
}
