#ifndef TRACKERGCS_H
#define TRACKERGCS_H

#include <pins_arduino.h>

#define minTrackingDistance 5 //Minimum distance in meters before tracking will start

//Telemetry variables
static float        uav_lat;                    // latitude
static float        uav_lon;                    // longitude
static uint8_t      uav_satellites_visible;     // number of satelites
static uint8_t      uav_fix_type;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static int16_t      uav_alt;                    // altitude (dm)
static int16_t      uav_heading;                // attitude heading
static int16_t      uav_gpsheading;               // gps heading

//home 
static float home_lon;
static float home_lat;
static int home_alt;
static int home_bearing;
static int home_dist;

//status 
static bool gps_fix;
static bool btholdstate;
static bool telemetry_ok;
static bool home_pos;
static bool home_bear;

//tracking 
static int Bearing;
static int Elevation;
static int servoBearing;
static int servoElevation;

static char trackingMode; // 0=RSSI-Tracking, 1=GPS-Tracking
static char protocolType; // 0=AeroQuad protocol, 1=Mikrokopter protocol

#endif
