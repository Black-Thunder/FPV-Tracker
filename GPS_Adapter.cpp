#include "GPS_Adapter.h"
#include "HWSerial.h"
#include <math.h>

uint8_t settingsArray[] = { 0x03, 0xFA, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t gpsConfigsSent;  // number of cfg msgs sent
uint8_t gpsConfigTimer;  // 0 = no more work, 1 = send now, >1 wait

GeodeticPosition currentPosition;

struct gpsData gpsData;

float cosLatitude = 0.7; // @ ~ 45 N/S, this will be adjusted to home loc

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

void setBaud(uint8_t baudSetting) {
	if (baudSetting == 0x12) GPS_SERIAL.begin(4800);
	if (baudSetting == 0x4B) GPS_SERIAL.begin(19200);
	if (baudSetting == 0x96) GPS_SERIAL.begin(38400);
	if (baudSetting == 0xE1) GPS_SERIAL.begin(57600);
	if (baudSetting == 0xC2) GPS_SERIAL.begin(115200);
	if (baudSetting == 0x84) GPS_SERIAL.begin(230400);
}

void calcChecksum(uint8_t *checksumPayload, uint8_t payloadSize) {
	uint8_t CK_A = 0, CK_B = 0;
	for (int i = 0; i < payloadSize; i++) {
		CK_A = CK_A + *checksumPayload;
		CK_B = CK_B + CK_A;
		checksumPayload++;
	}
	*checksumPayload = CK_A;
	checksumPayload++;
	*checksumPayload = CK_B;
}

void sendUBX(uint8_t *UBXmsg, uint8_t msgLength) {
	for (int i = 0; i < msgLength; i++) {
		GPS_SERIAL.write(UBXmsg[i]);
		GPS_SERIAL.flush();
	}
	GPS_SERIAL.println();
	GPS_SERIAL.flush();
}

uint8_t getUBX_ACK(uint8_t *msgID) {
	uint8_t CK_A = 0, CK_B = 0;
	uint8_t incoming_char;
	unsigned long ackWait = 0;
	uint8_t ackPacket[10] = { 0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int i = 0;
	while (1) {
		if (GPS_SERIAL.available()) {
			incoming_char = GPS_SERIAL.read();
			if (incoming_char == ackPacket[i]) {
				i++;
			}
			else if (i > 2) {
				ackPacket[i] = incoming_char;
				i++;
			}
		}
		if (i > 9) break;
		if (ackWait > 500) {
			return 5;
		}
		if (i == 4 && ackPacket[3] == 0x00) {
			return 1;
		}
		ackWait++;
	}

	for (i = 2; i < 8; i++) {
		CK_A = CK_A + ackPacket[i];
		CK_B = CK_B + CK_A;
	}
	if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
		return 10;
	}
	else {
		return 1;
	}
}

void configureUblox(uint8_t *settingsArrayPointer) {
	uint8_t gpsSetSuccess = 0;

	//Generate the configuration string for Navigation Mode
	uint8_t setNav[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&setNav[2], sizeof(setNav)-4);

	//Generate the configuration string for Data Rate
	uint8_t setDataRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };
	calcChecksum(&setDataRate[2], sizeof(setDataRate)-4);

	//Generate the configuration string for Baud Rate
	uint8_t setPortRate[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&setPortRate[2], sizeof(setPortRate)-4);

	uint8_t setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
	uint8_t setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
	uint8_t setGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
	uint8_t setRMC[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40 };
	uint8_t setVTG[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46 };

	while (gpsSetSuccess < 3)
	{
		sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
		if (gpsSetSuccess == 5) {
			gpsSetSuccess -= 4;
			setBaud(settingsArrayPointer[4]);
			uint8_t lowerPortRate[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5 };
			sendUBX(lowerPortRate, sizeof(lowerPortRate));
			GPS_SERIAL.begin(9600);
		}
		if (gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;
	while (gpsSetSuccess < 3) {
		sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function      
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;

	while (gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
		sendUBX(setGLL, sizeof(setGLL));
		gpsSetSuccess += getUBX_ACK(&setGLL[2]);
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;

	while (gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
		sendUBX(setGSA, sizeof(setGSA));
		gpsSetSuccess += getUBX_ACK(&setGSA[2]);
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;

	while (gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
		sendUBX(setGSV, sizeof(setGSV));
		gpsSetSuccess += getUBX_ACK(&setGSV[2]);
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;

	while (gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
		sendUBX(setRMC, sizeof(setRMC));
		gpsSetSuccess += getUBX_ACK(&setRMC[2]);
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	gpsSetSuccess = 0;

	while (gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
		sendUBX(setVTG, sizeof(setVTG));
		gpsSetSuccess += getUBX_ACK(&setVTG[2]);
		if (gpsSetSuccess == 5 || gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}

	gpsSetSuccess = 0;
	if (settingsArrayPointer[4] != 0x25) {
		sendUBX(&setPortRate[0], sizeof(setPortRate));
		setBaud(settingsArrayPointer[4]);
	}
}

// Initialize GPS subsystem (called once after powerup)
void initializeGps() {
	GPS_SERIAL.begin(57600);
	ubloxInit();
	initializeGpsData();
	configureUblox(settingsArray);
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
			GPS_SERIAL.begin(38400);
		}

		// ensure detection state (if we lost connection to GPS)
		gpsData.state = GPS_DETECTING;

		ubloxInit();

		// zero GPS state
		initializeGpsData();

		configureUblox(settingsArray);
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
