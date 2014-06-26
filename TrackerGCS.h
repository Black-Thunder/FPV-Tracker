#ifndef TRACKERGCS_H
#define TRACKERGCS_H

#include <LiquidCrystal.h>
#include <Servo.h>
#include <pins_arduino.h>

#define numberOfRSSISamples 10

#define minTrackingDistance 5					// Minimum distance in meters before tracking will start

#define verticalServoPin 10
#define horizontalServoPin 11
#define verticalServo 0
#define horizontalServo 1

#define GPSTrackingMode 0
#define RSSITrackingMode 1

#define AeroQuadProtocol 0
#define MikrokopterProtocol 1

#define battAREFValue         4.35					// V
#define battResistorHigh       10.1  				// kOhm
#define battResistorLow        1.48					// kOhm
const int battMonitorPin = A2;

// Telemetry variables
extern float			uavLatitude;                // latitude
extern float			uavLongitude;               // longitude
extern unsigned char    uavSatellitesVisible;		// number of satelites
extern int16_t			uavAltitude;                // altitude (dm)

// Home positioning data
extern float homeLongitude;
extern float homeLatitude;
extern float uavDistanceToHome;
extern unsigned int homeBearing;

// Status indicators
extern bool uavHasGPSFix;
extern bool isTelemetryOk;
extern long lastPacketReceived;

extern LiquidCrystal lcd;

// Tracking variables
extern unsigned int trackingBearing;
extern unsigned int trackingElevation;

const unsigned int verticalMin = 20;
const unsigned int verticalMid = 70;
const unsigned int verticalMax = 120;
const unsigned int horizontalMin = 0;
const unsigned int horizontalMid = 90;
const unsigned int horizontalMax = 180;
const int rssiTrackPin = A0;
const int rssiFixPin = A1;

extern unsigned char rssiTrack;
extern unsigned char rssiFix;
extern unsigned char rssiTrackOld;
extern unsigned char i;
extern unsigned char y;

extern unsigned char horizontalDirection;
extern unsigned char verticalDirection;

extern Servo VerticalServo;
extern Servo HorizontalServo;

extern unsigned char protocolType;

extern void applyServoCommand(int servo, unsigned int value);

#endif
