#include <stdint.h>
#include "GPS_Ublox.h"
#include "GPS_Adapter.h"
#include <Arduino.h>

ubloxState ubloxProcessDataState;
ublox_message ubloxMessage;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;
unsigned short ubloxClass, ubloxId;
unsigned char  ubloxCKA, ubloxCKB;

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
	unsigned long ackWait = millis();
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
		if ((millis() - ackWait) > 1500) {
			return 5;
		}
		if (i == 4 && ackPacket[3] == 0x00) {
			return 1;
		}
	}

	for (i = 2; i < 8; i++) {
		CK_A = CK_A + ackPacket[i];
		CK_B = CK_B + CK_A;
	}
	if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
		return 10;
	}
	else {
		delay(1000);
		return 1;
	}
}

bool configureUbloxGPS() {
	uint8_t gpsSetSuccess = 0;
	int retryCounter = 0;

	//UART1, 9600 baud, out: UBX, in: UBX+NMEA+RTCM
	uint8_t setPORTMsg[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&setPORTMsg[2], sizeof(setPORTMsg)-4);

	//2Hz update rate
	uint8_t setRATEMsg[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };
	calcChecksum(&setRATEMsg[2], sizeof(setRATEMsg)-4);

	//Disable NAV-STATUS msg
	uint8_t disableNavStatusMsg[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&disableNavStatusMsg[2], sizeof(disableNavStatusMsg)-4);

	//Enable NAV-SOL msg
	uint8_t enableNavSOLMsg[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&enableNavSOLMsg[2], sizeof(enableNavSOLMsg)-4);

	//Enable NAV-POSLLH msg
	uint8_t enableNavPOSLLHMsg[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&enableNavPOSLLHMsg[2], sizeof(enableNavPOSLLHMsg)-4);

	//Stationary mode, fix mode: auto
	uint8_t setNAV5Msg[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00,
		0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	calcChecksum(&setNAV5Msg[2], sizeof(setNAV5Msg)-4);

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(setPORTMsg, sizeof(setPORTMsg));
		gpsSetSuccess = getUBX_ACK(&setPORTMsg[2]);
		retryCounter++;
	}
	if (retryCounter == 3) return false;
	gpsSetSuccess = 0;
	retryCounter = 0;

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(setRATEMsg, sizeof(setRATEMsg));
		gpsSetSuccess = getUBX_ACK(&setRATEMsg[2]);
	}
	if (retryCounter == 3) return false;
	gpsSetSuccess = 0;
	retryCounter = 0;

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(disableNavStatusMsg, sizeof(disableNavStatusMsg));
		gpsSetSuccess = getUBX_ACK(&disableNavStatusMsg[2]);
	}
	if (retryCounter == 3) return false;
	gpsSetSuccess = 0;
	retryCounter = 0;

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(enableNavSOLMsg, sizeof(enableNavSOLMsg));
		gpsSetSuccess = getUBX_ACK(&enableNavSOLMsg[2]);
	}
	if (retryCounter == 3) return false;
	gpsSetSuccess = 0;
	retryCounter = 0;

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(enableNavPOSLLHMsg, sizeof(enableNavPOSLLHMsg));
		gpsSetSuccess = getUBX_ACK(&enableNavPOSLLHMsg[2]);
	}
	if (retryCounter == 3) return false;
	gpsSetSuccess = 0;
	retryCounter = 0;

	while (gpsSetSuccess != 10 && retryCounter < 3) {
		sendUBX(setNAV5Msg, sizeof(setNAV5Msg));
		gpsSetSuccess = getUBX_ACK(&setNAV5Msg[2]);
	}
	if (retryCounter == 3) return false;

	return true;
}

// Initialize parser
bool ubloxInit() {
	ubloxProcessDataState = WAIT_SYNC1;
	return configureUbloxGPS();
}

// process complete binary packet
void ubloxParseData() {// uses publib vars
	gpsData.sentences++;

	if (ubloxClass == 1) { // NAV
		if (ubloxId == 2) { // NAV:POSLLH
			gpsData.lat = ubloxMessage.nav_posllh.lat;
			gpsData.lon = ubloxMessage.nav_posllh.lon;
		}
		else if (ubloxId == 6) { // NAV:SOL
			gpsData.sats = ubloxMessage.nav_sol.numSV;

			switch (ubloxMessage.nav_sol.gpsFix) {
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
