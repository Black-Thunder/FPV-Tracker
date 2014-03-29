#ifndef TRACKERGCS_H
#define TRACKERGCS_H

#include <LiquidCrystal.h>
#include <Servo.h>
#include <pins_arduino.h>

#define minTrackingDistance 5					// Minimum distance in meters before tracking will start
#define invalidPositionCoordinate 0x7FFFFFFF
#define invalidAltitude 65535

#define verticalServoPin 10
#define horizontalServoPin 11
#define verticalServo 0
#define horizontalServo 1

#define GPSTrackingMode 0
#define RSSITrackingMode 1

#define AeroQuadProtocol 0
#define MikrokopterProtocol 1

#define BATT_AREF         4.75		// V
#define BATT_R_HIGH       10.0  	// kOhm
#define BATT_R_LOW        1.498		// kOhm
const int battMonitorPin = A2;

// Telemetry variables
extern float        uavLatitude;                // latitude
extern float        uavLongitude;               // longitude
extern uint8_t      uavSatellitesVisible;	// number of satelites
extern int16_t      uavAltitude;                // altitude (dm)

// Home positioning data
extern float homeLongitude;
extern float homeLatitude;
extern float uavDistanceToHome;
extern int homeBearing;

// Status indicators
extern bool uavHasGPSFix;
extern bool isTelemetryOk;
extern long lastPacketReceived;

extern LiquidCrystal lcd;

// Tracking variables
extern int trackingBearing;
extern int trackingElevation;

const int verticalMin = 20;
const int verticalMid = 70;
const int verticalMax = 120;
const int horizontalMin = 0;
const int horizontalMid = 85;
const int horizontalMax = 170;
const int rssi1 = A0;
const int rssi2 = A1;

extern int rssiTrack;
extern int rssiFix;
extern int rssiTrackOld;
extern int i;
extern int y;

extern char horizontalDirection;
extern char verticalDirection;

extern Servo VerticalServo;
extern Servo HorizontalServo;

extern char protocolType;

extern void applyServoCommand(int servo, int value);

#endif
