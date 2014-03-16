#ifndef MIKROKOPTER_DATASTRUCTS_H
#define MIKROKOPTER_DATASTRUCTS_H

#define REQUEST_OSD_DATA "#bo?]==EG\r"

#define u8 uint8_t
#define s8 int8_t
#define u16 uint16_t
#define s16 int16_t
#define u32 uint32_t
#define s32 int32_t

typedef struct {
    s32 Longitude; // in 1E-7 deg
    s32 Latitude; // in 1E-7 deg
    s32 Altitude; // in mm
    u8 Status; // validity of data
} __attribute__((packed)) GPS_Pos_t;

typedef struct {
    u16 Distance; // distance to target in dm
    s16 Bearing; // course to target in deg
} __attribute__((packed)) GPS_PosDev_t;

typedef struct {
    u8 Version; // version of the data structure
    GPS_Pos_t CurrentPosition; // see ubx.h for details
    GPS_Pos_t TargetPosition;
    GPS_PosDev_t TargetPositionDeviation;
    GPS_Pos_t HomePosition;
    GPS_PosDev_t HomePositionDeviation;
    u8 WaypointIndex; // index of current waypoints running from 0 to WaypointNumber-1
    u8 WaypointNumber; // number of stored waypoints
    u8 SatsInUse; // number of satellites used for position solution
    s16 Altimeter; // hight according to air pressure
    s16 Variometer; // climb(+) and sink(-) rate
    u16 FlyingTime; // in seconds
    u8 UBat; // Battery Voltage in 0.1 Volts
    u16 GroundSpeed; // speed over ground in cm/s (2D)
    s16 Heading; // current flight direction in Â° as angle to north
    s16 CompassHeading; // current compass value in Â°
    s8 AngleNick; // current Nick angle in 1Â°
    s8 AngleRoll; // current Rick angle in 1Â°
    u8 RC_Quality; // RC_Quality
    u8 FCFlags; // Flags from FC
    u8 NCFlags; // Flags from NC
    u8 Errorcode; // 0 --> okay
    u8 OperatingRadius; // current operation radius around the Home Position in m
    s16 TopSpeed; // velocity in vertical direction in cm/s
    u8 TargetHoldTime; // time in s to stay at the given target, counts down to 0 if target has been reached
    u8 FCStatusFlags2; // StatusFlags2 (since version 5 added)
    s16 SetpointAltitude; // setpoint for altitude
    u8 Gas; // for future use
    u16 Current; // actual current in 0.1A steps
    u16 UsedCapacity; // used capacity in mAh
} __attribute__((packed)) NaviData_t;

#endif
