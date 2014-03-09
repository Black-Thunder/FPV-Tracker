#ifndef TRACKERGCS_H
#define TRACKERGCS_H

#include <pins_arduino.h>

#define minTrackingDistance 5 //Minimum distance in meters before tracking will start

//Telemetry variables
static float        uav_lat;                    // latitude
static float        uav_lon;                    // longitude
static uint8_t      uav_satellites_visible;     // number of satelites
static int16_t      uav_alt;                    // altitude (dm)
static int16_t      uav_heading;                // attitude heading
static int16_t      uav_gpsheading;             // gps heading

//home 
static float home_lon;
static float home_lat;
static float home_dist;
static int home_alt;
static int home_bearing;

//status 
static bool hasGPSFix;
static bool isTelemetryOk;

//tracking 
static int Bearing;
static int Elevation;

static char trackingMode; // 0=RSSI-Tracking, 1=GPS-Tracking
static char protocolType; // 0=AeroQuad protocol, 1=Mikrokopter protocol

#endif
