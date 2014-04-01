#ifndef GPSUBLOX_H
#define GPSUBLOX_H

#include <stdint.h>
#include "GPS_DataType.h"

// UBLOX binary message definitions
struct ublox_NAV_POSLLH { // 01 02 (28)
	uint32_t iTow;
	int32_t lon; // 1e-7 degrees
	int32_t lat; // 1e-7 degrees
	int32_t height; // mm
	int32_t hMSL; // mm
	uint32_t hAcc; //mm
	uint32_t vAcc; //mm
};

struct ublox_NAV_SOL { // 01 6 (52)
	uint32_t iTow;
	int32_t  fTow;
	int16_t  week;
	uint8_t  gpsFix;
	uint8_t  flags;
	int32_t  ecefX;
	int32_t  ecefY;
	int32_t  ecefZ;
	int32_t  pAcc;
	int32_t  ecefVX;
	int32_t  ecefVY;
	int32_t  ecefVZ;
	int32_t  sAcc;
	uint16_t pDOP;
	uint8_t  res1;
	uint8_t  numSV;
	uint32_t res2;
};

struct ublox_NAV_VELNED{ // 01 12h (36)
	uint32_t iTow;
	int32_t  velN; // cm/s
	int32_t  velE; // cm/s
	int32_t  velD; // cm/s
	uint32_t  speed; // cm/s
	uint32_t  gSpeed; // cm/s
	int32_t  heading; // dev 1e-5
	uint32_t sAcc; // cm/s
	uint32_t cAcc; // deg 1e-5
};

typedef union {
	struct ublox_NAV_POSLLH nav_posllh;
	struct ublox_NAV_VELNED nav_velned;
	struct ublox_NAV_SOL nav_sol;
	unsigned char raw[52];
} ublox_message;

enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB };

bool ubloxInit();
void ubloxParseData();
int ubloxProcessData(unsigned char data);

#endif
