#ifndef AEROQUAD_DATASTRUCTS_H
#define AEROQUAD_DATASTRUCTS_H

typedef struct {
	unsigned short  id;
	long  latitude;
	long  longitude;
	short altitude;
	short course;
	short heading;
	uint8_t  speed;
	uint8_t  rssi;
	uint8_t  voltage;
	uint8_t  current;
	unsigned short capacity;
	unsigned short gpsinfo;
	uint8_t  ecc[8];
}
__attribute__((packed)) TelemetryPacket_t;

#endif


