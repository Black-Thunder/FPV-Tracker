#ifndef TRACKERGCS_H
#define TRACKERGCS_H

#include <LiquidCrystal.h>
#include <Servo.h>
#include <pins_arduino.h>

#define minTrackingDistance 5 //Minimum distance in meters before tracking will start

// Telemetry variables
extern float        uav_lat;                    // latitude
extern float        uav_lon;                    // longitude
extern uint8_t      uav_satellites_visible;     // number of satelites
extern int16_t      uav_alt;                    // altitude (dm)

// Home positioning data
extern float home_lon;
extern float home_lat;
extern float home_dist;
extern int home_alt;
extern int home_bearing;

// Status indicators
extern bool hasGPSFix;
extern bool isTelemetryOk;
extern long lastPacketReceived;

extern LiquidCrystal lcd;

// Tracking variables
extern int Bearing;
extern int Elevation;

const int verticalMin = 20;
const int verticalMid = 70;
const int verticalMax = 120;
const int horizontalMin = 0;
const int horizontalMid = 90;
const int horizontalMax = 110;
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

extern char protocolType; // 0=AeroQuad protocol, 1=Mikrokopter protocol

#endif
