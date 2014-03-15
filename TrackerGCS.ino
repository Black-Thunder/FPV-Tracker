#include <LiquidCrystal.h>
#include <Servo.h>
#include "TrackerGCS.h"
#include "RSSI_Tracker.h"
#include "GPS_Tracker.h"
#include "GPS_Adapter.h"
#include "HMC5883L.h"
#include "uart1.h"
#include "Mikrokopter_Datastructs.h"
#include "AeroQuad_Datastructs.h"

// RSSI tracking
const int horizontalTolerance = 2;
const int thresholdValue = 80;

int calibrate1 = 0;
int calibrate2 = 0;
int i = horizontalMid;
int y = verticalMid;

char horizontalDirection = 0;
char verticalDirection = 0;

#define NUMBER_OF_SAMPLES 10

// GPS tracking
char trackingMode = 0; // 1=RSSI-Tracking, 0=GPS-Tracking
char protocolType = 0;
const int protocolTypeSwitchPin = 8; // Switch to determine protocol type on start-up; LOW=AeroQuad, HIGH=Mikrokopter
bool lastProtocolTypeSwitchState = LOW;

float uavLatitude = invalidPositionCoordinate;
float uavLongitude = invalidPositionCoordinate;
uint8_t uavSatellitesVisible = 0;
int16_t uavAltitude = invalidAltitude;

float homeLongitude = invalidPositionCoordinate;
float homeLatitude = invalidPositionCoordinate;
float uavDistanceToHome = 0;
int homeBearing = 0;

int trackingBearing = 0;
int trackingElevation = 0;

bool uavHasGPSFix = false;
bool isTelemetryOk = false;
long lastPacketReceived = 0;

// General
const int trackingModeSwitchPin = 9; // Switch to determine tracking mode on start-up; LOW=RSSI, HIGH=GPS
bool lastTrackingModeSwitchState = LOW;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

int rssiTrack = 0;
int rssiFix = 0;
int rssiTrackOld = 0;

Servo VerticalServo;
Servo HorizontalServo;

// Main loop time variable
unsigned long frameCounter = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100

void setup()
{
	//TODO remove/debug
	Serial.begin(115200);

	lcd.begin(16, 2);

        determineTrackingMode();

	VerticalServo.attach(10);
	VerticalServo.write(verticalMid);
	HorizontalServo.attach(11);
	HorizontalServo.write(horizontalMid);

	calibrateRSSI();
}

void calibrateRSSI() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Calibrating...");

	for (int counter = 0; counter < NUMBER_OF_SAMPLES; counter++) {
		calibrate1 = calibrate1 + analogRead(rssi1);
		delay(50);
	}
	calibrate1 = calibrate1 / NUMBER_OF_SAMPLES;

	for (int counter = 0; counter < NUMBER_OF_SAMPLES; counter++) {
		calibrate2 = calibrate2 + analogRead(rssi2);
		delay(50);
	}
	calibrate2 = calibrate2 / NUMBER_OF_SAMPLES;
}

void setupGPSTrackingMode() {
        determineProtocolType();
	delay(1000); // Keep LCD message visible

	usart1_init();

	//already done by OSD
	//usart1_request_nc_uart();

	initializeGps();
	setupHMC5883L();  
}

void setupHMC5883L(){
	compass = HMC5883L();
	compass.CheckConnectionState();

	if (compass.isMagDetected) {
		compass.SetScale(1.3); //Set the scale of the compass.
		compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	}
	else {
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Mag Failure!");
		lcd.setCursor(0, 1);
		lcd.print("Heading set to 0");

		delay(2000);  // Keep LCD message visible
	}
}

void determineTrackingMode() {
	if (digitalRead(trackingModeSwitchPin) == HIGH) {
		trackingMode = 1;
                lastTrackingModeSwitchState = HIGH;
                
                lcd.setCursor(0, 0);
		lcd.print("Mode: RSSI");
		delay(1000); // Keep LCD message visible
	}
	else {
		trackingMode = 0;
                lastTrackingModeSwitchState = LOW;
                
                lcd.setCursor(0, 0);
		lcd.print("Mode: GPS");

                setupGPSTrackingMode();
	}
}

void determineProtocolType() {
	if (digitalRead(protocolTypeSwitchPin) == HIGH) {
		protocolType = 1;
                lastProtocolTypeSwitchState = HIGH;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: MK");
	}
	else {
		protocolType = 0;
                lastProtocolTypeSwitchState = LOW;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: AQ");
	}
}

void loop()
{
	currentTime = micros();
	deltaTime = currentTime - previousTime;

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) {
		process100HzTask();

		frameCounter++;
		previousTime = currentTime;
	}

	if (frameCounter % TASK_10HZ == 0) {   //   10 Hz tasks
		process10HzTask();
	}

	if (frameCounter % TASK_5HZ == 0) {  //  5 Hz tasks
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
	if (trackingMode == 0) {
		updateGps();
	}
}

void process10HzTask() {
	if (trackingMode == 0) {
		// request OSD Data from NC every 100ms, already requested by OSD
		//usart1_puts_pgm(PSTR(REQUEST_OSD_DATA));

		if (millis() - lastPacketReceived > 2000) {
			isTelemetryOk = false;
		}

		processUsart1Data();
	}
}

void process5HzTask() {
	if (trackingMode == 0) {
		updateGCSPosition();

		if (compass.isMagDetected) {
			updateGCSHeading();
		}
		else {
			homeBearing = 0;
		}
	}

	processTracking();
}

void process1HzTask() {
	updateLCD();
        checkSwitchState();
}

void processTracking() {
	readRSSI();

	if (trackingMode == 1) {
	        if (rssiTrack <= thresholdValue) {
		        calculateRSSIDiff();

			if (rssiDiv <= horizontalTolerance) {
				if (rssiTrack <= 45) {
					VerticalServo.write(verticalMid);

					if (i >= horizontalMid) {
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
	}
	else if (trackingMode == 0) {
		// Only move servo if home position is set, otherwise standby to last known position
		if (isHomePositionSet && isTelemetryOk) {
			calculateTrackingVariables(homeLongitude, homeLatitude, uavLongitude, uavLatitude, uavAltitude); //calculate tracking bearing/azimuth

			//set current GPS bearing relative to homeBearing
			if (trackingBearing >= homeBearing) {
				trackingBearing -= homeBearing;
			}
			else {
				trackingBearing += 360 - homeBearing;
			}

			if (uavDistanceToHome > minTrackingDistance) {
				servoPathfinder(trackingBearing, trackingElevation);
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
	lcd.print("RSSI Fix   ");
	lcd.print(rssiFix);
}

void checkSwitchState() {
        if(digitalRead(trackingModeSwitchPin) != lastTrackingModeSwitchState) {
                lcd.clear();
        
                determineTrackingMode();
        }
        if(trackingMode == 0 && digitalRead(protocolTypeSwitchPin) != lastProtocolTypeSwitchState) {
                lcd.clear();
        
                determineProtocolType(); 
        } 
}

void readRSSI() {
	rssiTrackOld = rssiTrack;

	rssiTrack = map(analogRead(rssi1), 0, calibrate1, 0, 100);
	rssiFix = map(analogRead(rssi2), 0, calibrate2, 0, 100);

	rssiTrack = constrain(rssiTrack, 0, 100);
	rssiFix = constrain(rssiFix, 0, 100);
}
