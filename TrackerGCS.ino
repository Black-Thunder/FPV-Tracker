#include <LiquidCrystal.h>
#include <Servo.h>
#include "TrackerGCS.h"
#include "uart.h"
#include "Mikrokopter_Datastructs.h"
#include "GpsAdapter.h"
#include "Compass.h"
#include "Magnetometer_HMC5883L.h"

static const int horizontalTolerance = 2;
static const int thresholdValue = 80;
static const int verticalMin = 20;
static const int verticalMid = 70;
static const int verticalMax = 120;
static const int horizontalMin = 0;
static const int horizontalMid = 90;
static const int horizontalMax = 110;
int trackingCounter = 0;
int calibrate1 = 0;
int calibrate2 = 0;

// main loop time variable
unsigned long frameCounter = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100

#define NUMBER_OF_SAMPLES 10

static const int rssi1 = A0;
static const int rssi2 = A1;
int rssiTrack = 0;
int rssiFix = 0;
int rssiTrackOld = 0;
int rssiDiv = 0;
int i = horizontalMid;
int y = verticalMid;

char horizontalDirection;
char verticalDirection;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

Servo VerticalServo;
Servo HorizontalServo;

void setup()
{
  determineTrackingMode();

  lcd.begin(16, 2);
  VerticalServo.attach(10);
  VerticalServo.write(verticalMid);
  HorizontalServo.attach(11);
  HorizontalServo.write(horizontalMid);

  if(trackingMode == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Calibrating...");

    for(i = 0; i < NUMBER_OF_SAMPLES; i++) {
      calibrate1 = calibrate1 + analogRead(rssi1);
      delay(25);
    }
    calibrate1 = calibrate1 / NUMBER_OF_SAMPLES;

    for(i = 0; i < NUMBER_OF_SAMPLES; i++) {
      calibrate2 = calibrate2 + analogRead(rssi2);
      delay(25);
    }
    calibrate2 = calibrate2 / NUMBER_OF_SAMPLES;
  }
  else if(trackingMode == 1) {
    determineProtocolType();
    usart0_Init();

    //already done by OSD
    //usart0_request_nc_uart();

    initializeGps();
  }
}

void determineTrackingMode() {
  trackingMode = 1; 
}

void determineProtocolType() {
  protocolType = 1; 
}

void loop()
{
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {
    frameCounter++;

    process100HzTask();

    if(frameCounter % TASK_50HZ == 0) {
      process50HzTask();  
    }

    previousTime = currentTime;
  }

  if(frameCounter % TASK_10HZ == 0) {   //   10 Hz tasks
    process10HzTask();
  }

  if(frameCounter % TASK_5HZ == 0) {  //  5 Hz tasks
    process5HzTask();
  }

  if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
    process1HzTask();
  }

  if (frameCounter >= 100) {
    frameCounter = 0;
  }
}

void process100HzTask() {
  updateGps();
  updateGCSPosition();
}

void process50HzTask() { 
}

void process10HzTask() {
  if(trackingMode == 1) {
    // request OSD Data from NC every 100ms, already requested by OSD
    //usart0_puts_pgm(PSTR(REQUEST_OSD_DATA));

    processUsartData();
  }
}

void process5HzTask() {
  processTracking(); 
}

void process1HzTask() {
  updateLCD();
}

void processTracking() {
  readRSSI();

  if(trackingMode == 0) {
    if(rssiTrack <= thresholdValue) {        
      if(trackingCounter < 15) {
        trackingCounter++;
        calculateRSSIDiff();

        if(rssiDiv <= horizontalTolerance ) {   
          if(rssiTrack <= 45) {
            VerticalServo.write(verticalMid);

            if(i >= horizontalMid) {
              i = i - 30;
              horizontalDirection = 'L';
            }
            else {
              i = i + 30;
              horizontalDirection = 'R';
            }
          }
          else {               
            trackVertical();
          }
        }
        else {   
          trackHorizontal();
        }
      }
      else {
        HorizontalServo.write(horizontalMid);
        VerticalServo.write(verticalMid);
      }
    }
    else {
      trackingCounter = 0; 
    }
  }
  else if(trackingMode == 1) {
    //only move servo if gps has a 3D fix, or standby to last known position.
    if (gps_fix && telemetry_ok) {

      int rel_alt = uav_alt - home_alt; // relative altitude to ground in decimeters
      calc_tracking( home_lon, home_lat, uav_lon, uav_lat, rel_alt); //calculate tracking bearing/azimuth
      //set current GPS bearing relative to home_bearing

      if(Bearing >= home_bearing){
        Bearing-=home_bearing;
      }
      else
      {
        Bearing+=360-home_bearing;
      }

      if(home_dist > minTrackingDistance) { //don't track when <10m 

      }
    }  
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RSSI Track ");
  lcd.print(rssiTrack);
  lcd.setCursor(0, 1);
  lcd.print("RSSI Fix ");
  lcd.print(rssiFix);
}

void readRSSI() {
  rssiTrackOld = rssiTrack;

  rssiTrack = map(analogRead(rssi1), 0, calibrate1, 0, 100);
  rssiFix = map(analogRead(rssi2), 0, calibrate2, 0, 100);

  rssiTrack = constrain(rssiTrack, 0, 100);
  rssiFix = constrain(rssiFix, 0, 100);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              RSSI Tracker                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculateRSSIDiff() {
  rssiDiv = (rssiTrack - rssiTrackOld);

  if (rssiDiv < 0) {
    rssiDiv = rssiDiv * -1;
  }
}

void trackHorizontal() {    
  if(rssiTrack > rssiTrackOld) {
    if (horizontalDirection == 'L') {
      i = i + 10;
      horizontalDirection = 'L';
    }
    else {
      i = i - 10;
      horizontalDirection = 'R';
    }
  }
  else {
    if (horizontalDirection == 'R') {
      i = i + 10;
      horizontalDirection = 'L';
    }
    else {
      i = i - 10;
      horizontalDirection = 'R';
    }
  }      

  HorizontalServo.write(i);

  if (i <= horizontalMin || i >= horizontalMax) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reset");

    i = horizontalMid;
    HorizontalServo.write(horizontalMid);
    VerticalServo.write(verticalMid);
    return;
  }
}


void trackVertical() {
  if(rssiTrack > rssiTrackOld) {
    if (verticalDirection == 'O')
    {
      y = y - 5;
      verticalDirection = 'O';
    }
    else {
      y = y + 5;
      verticalDirection = 'U';
    }
  }
  else {
    if (verticalDirection == 'U') {
      y = y - 5;
      verticalDirection = 'O';
    }
    else {
      y = y + 5;
      verticalDirection = 'U';
    }
  }        

  VerticalServo.write(y);

  if (y <= verticalMin || y >= verticalMax) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reset");

    y = verticalMid;
    return;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               GPS Tracker                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateGCSPosition() {

}


void servoPathfinder(int angle_b, int angle_a){   // ( bearing, elevation )
  //find the best way to move pan servo considering 0° reference face toward
  if (angle_b<=180) {
    if (horizontalMax >= angle_b ) {
      //works for all config
      //define limits
      if (angle_a <= verticalMin) {
        // checking if we reach the min tilt limit
        angle_a = verticalMin;
      }
      else if (angle_a > verticalMax) {
        //shouldn't happend but just in case
        angle_a = verticalMax; 
      }

    }
    else if (horizontalMax < angle_b ) {
      //relevant for 180° tilt config only, in case bearing is superior to pan_maxangle

      angle_b = 360-(180-angle_b);
      if (angle_b >= 360) {
        angle_b = angle_b - 360;
      }

      // invert pan axis 
      if (verticalMax >= ( 180-angle_a )) {
        // invert pan & tilt for 180° Pan 180° Tilt config

        angle_a = 180-angle_a;

      }
      else if (verticalMax < ( 180-angle_a )) {
        // staying at nearest max pos
        angle_a = verticalMax;
      }
    }
  }

  else if ( angle_b > 180 )
    if(horizontalMin > 360-angle_b ) {
      //works for all config
      if (angle_a < verticalMin) {
        // checking if we reach the min tilt limit
        angle_a = verticalMin;
      }
    }
    else if (horizontalMin <= 360-angle_b ) {
      angle_b = angle_b - 180;
      if (verticalMax >= ( 180-angle_a )) {
        // invert pan & tilt for 180/180 conf
        angle_a = 180-angle_a;
      }
      else if (verticalMax < ( 180-angle_a)) {
        // staying at nearest max pos
        angle_a = verticalMax;
      }
    }

  HorizontalServo.write(angle_b);
  VerticalServo.write(angle_a);
}

void calc_tracking(float lon1, float lat1, float lon2, float lat2, int alt) {
  //// (homelon, homelat, uavlon, uavlat, uavalt ) 
  //// Return Bearing & Elevation angles in degree
  //  float a, tc1, R, c, d, dLat, dLon;
  // 
  // // converting to radian
  lon1=toRad(lon1);
  lat1=toRad(lat1);
  lon2=toRad(lon2);
  lat2=toRad(lat2);
  // 
  // 
  // //calculating bearing in degree decimal
  Bearing = calc_bearing(lon1,lat1,lon2,lat2);
  //
  //
  ////calculating distance between uav & home
  Elevation = calc_elevation(lon1,lat1,lon2,lat2,alt);
}


int calc_bearing(float lon1, float lat1, float lon2, float lat2) {
  // bearing calc, feeded in radian, output degrees
  float a;
  //calculating bearing 
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  a=toDeg(a);
  if (a<0) a=360+a;
  int b = (int)round(a);
  return b;
}

int calc_elevation(float lon1, float lat1, float lon2, float lat2, int alt) {
  // feeded in radian, output in degrees
  float a, el, c, d, R, dLat, dLon;
  //calculating distance between uav & home
  R=6371000.0;    //in meters. Earth radius 6371km
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  c = 2* asin(sqrt(a));  
  d =(R * c);
  home_dist = d/10;
  el=atan((float)alt/(10*d));// in radian
  el=toDeg(el); // in degree
  int b = (int)round(el);
  return b;
}

float toRad(float angle) {
  // convert degrees to radians
  angle = angle*0.01745329; // (angle/180)*pi
  return angle;
}

float toDeg(float angle) {
  // convert radians to degrees.
  angle = angle*57.29577951;   // (angle*180)/pi
  return angle;
}











