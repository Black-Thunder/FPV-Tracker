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

#define battAREFValue          5.0				// Papa: 4.35 Ich: 5.0
#define battResistorHigh       10.1  				
#define battResistorLow        1.48				
const int battMonitorPin = A2;

// Telemetry variables
extern int32_t			uavLatitude;                // latitude
extern int32_t			uavLongitude;               // longitude
extern unsigned char    uavSatellitesVisible;		// number of satelites
extern int16_t			uavAltitude;                // altitude (dm)

// Home positioning data
extern int32_t homeLongitude;
extern int32_t homeLatitude;
extern int32_t uavDistanceToHome;
extern unsigned int homeBearing;

// Status indicators
extern bool uavHasGPSFix;
extern bool isTelemetryOk;
extern long lastPacketReceived;

extern LiquidCrystal lcd;

// Tracking variables
extern unsigned int trackingBearing;
extern unsigned int trackingElevation;

const unsigned int verticalMin = 0;
const unsigned int verticalMid = 60;
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
