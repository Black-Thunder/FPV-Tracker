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
char protocolType = AeroQuadProtocol;
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
char trackingMode = GPSTrackingMode;
const int trackingModeSwitchPin = 9; // Switch to determine tracking mode on start-up; LOW=RSSI, HIGH=GPS
bool lastTrackingModeSwitchState = LOW;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

int rssiTrack = 0;
int rssiFix = 0;
int rssiTrackOld = 0;

int servoCommands[2] = { verticalMid, horizontalMid };
int previousServoCommands[2] = { -1, -1 };

Servo VerticalServo;
Servo HorizontalServo;

float battVoltage = 0;
bool isBattLow = false;

// Main loop time variable
unsigned long frameCounter = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100

void setup() {
	//TODO remove/debug
	Serial.begin(115200);

	lcd.begin(16, 2);

	determineTrackingMode();

	VerticalServo.attach(verticalServoPin);
	HorizontalServo.attach(horizontalServoPin);

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
	delay(2000); // Keep LCD message visible

	usart1_init();
	usart1_DisableTXD();
	sei();

	//request NC uart from MK, already done by C-OSD/Smart-OSD
	//if (protocolType == MikrokopterProtocol) {
	//	usart1_request_nc_uart();
	//}

	lcd.setCursor(0, 1);
	lcd.print("Configuring GPS");
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
		trackingMode = RSSITrackingMode;
		lastTrackingModeSwitchState = HIGH;

		lcd.setCursor(0, 0);
		lcd.print("Mode: RSSI");
		delay(1000); // Keep LCD message visible
	}
	else {
		trackingMode = GPSTrackingMode;
		lastTrackingModeSwitchState = LOW;

		lcd.setCursor(0, 0);
		lcd.print("Mode: GPS");

		setupGPSTrackingMode();
	}
}

void determineProtocolType() {
	if (digitalRead(protocolTypeSwitchPin) == HIGH) {
		protocolType = MikrokopterProtocol;
		lastProtocolTypeSwitchState = HIGH;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: MK");
	}
	else {
		protocolType = AeroQuadProtocol;
		lastProtocolTypeSwitchState = LOW;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: AQ");
	}
}

void loop() {
	currentTime = micros();
	deltaTime = currentTime - previousTime;

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) {
		process100HzTask();

		// ================================================================
		// 50Hz task loop
		// ================================================================
		if (frameCounter % TASK_50HZ == 0) { // 50 Hz tasks
			process50HzTask();
		}

		frameCounter++;

		// ================================================================
		// 10Hz task loop
		// ================================================================
		if (frameCounter % TASK_10HZ == 0) {  // 10 Hz tasks
			process10HzTask();
		}

		// ================================================================
		// 5Hz task loop
		// ================================================================
		if (frameCounter % TASK_5HZ == 0) {  //  5 Hz tasks
			process5HzTask();
		}

		// ================================================================
		// 1Hz task loop
		// ================================================================
		if (frameCounter % TASK_1HZ == 0) {  //  1 Hz tasks
			process1HzTask();
		}

		previousTime = currentTime;
	}

	if (frameCounter >= 100) {
		frameCounter = 0;
	}

}

void process100HzTask() {
	if (trackingMode == GPSTrackingMode) {
		updateGps();
	}
}

void process50HzTask() {
	writeServos();
}

void process10HzTask() {
	if (trackingMode == GPSTrackingMode) {
		// Request data from MK, this is already done by C-OSD/Smart-OSD
		//if (protocolType == MikrokopterProtocol) {
		//	requestMikrokopterTelemetryData();
		//}

		processUsart1Data();

		// No telemetry data received for more than 2 seconds
		if (millis() - lastPacketReceived > 2000) {
			isTelemetryOk = false;
		}
	}

        measureBatteryVoltage();
}

void process5HzTask() {
	if (trackingMode == GPSTrackingMode) {
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

	if (trackingMode == RSSITrackingMode) {
		if (rssiTrack <= thresholdValue) {
			calculateRSSIDiff();

			if (rssiDiv <= horizontalTolerance) {
				if (rssiTrack <= 45) {
					applyServoCommand(verticalServo, verticalMid);

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
	else if (trackingMode == GPSTrackingMode) {
		// Only move servo if home position is set, otherwise standby to last known position
		if (isHomePositionSet && isTelemetryOk) {
			calculateTrackingVariables(homeLongitude, homeLatitude, uavLongitude, uavLatitude, uavAltitude);

                        if(uavDistanceToHome > minTrackingDistance) {
			//set current GPS bearing relative to homeBearing
			if (trackingBearing >= homeBearing) {
				trackingBearing -= homeBearing;
			}
			else {
				trackingBearing += 360 - homeBearing;
			}

			if(trackingBearing >= 0 && trackingBearing <= 90) {
                                trackingBearing = map(trackingBearing, 90, 0, horizontalMin, horizontalMid);
                        }
			else if(trackingBearing >= 270 && trackingBearing <= 360) {
                                trackingBearing = map(trackingBearing, 360, 270, horizontalMid, horizontalMax);
                        }
			else if(trackingBearing > 90 && trackingBearing < 180) {
                                trackingBearing = horizontalMin;
                        }
			else {
                                trackingBearing = horizontalMax;
                        }
                        
                        trackingElevation = map(trackingElevation, 0, 90, 0, 180);

			applyServoCommand(horizontalServo, trackingBearing);
			applyServoCommand(verticalServo, trackingElevation);
                        } 
		}
	}
}

void applyServoCommand(int servo, int value) {
	if (servo > horizontalServo) return;

	if (servo == verticalServo) {
		value = constrain(value, verticalMin, verticalMax);
	}
	else if (servo == horizontalServo) {
		value = constrain(value, horizontalMin, horizontalMax);
	}

	previousServoCommands[servo] = servoCommands[servo];
	servoCommands[servo] = value;
}

void writeServos() {
	if (previousServoCommands[verticalServo] != servoCommands[verticalServo]) {
		VerticalServo.write(servoCommands[verticalServo]);
		previousServoCommands[verticalServo] = servoCommands[verticalServo];
	}

	if (previousServoCommands[horizontalServo] != servoCommands[horizontalServo]) {
		HorizontalServo.write(servoCommands[horizontalServo]);
		previousServoCommands[horizontalServo] = servoCommands[horizontalServo];
	}
}

void updateLCD() {
	lcd.clear();
	lcd.setCursor(0, 0);
        
        if(isBattLow) {
                lcd.print("! LOW VOLTAGE !");
        }     
	else if (trackingMode == GPSTrackingMode) {
          	if (rssiTrack == 100 && rssiFix == 100) {
			// Would be a total of 17 chars
			lcd.print("Track ");
			lcd.print(rssiTrack);
			lcd.print("Fix ");
			lcd.print(rssiFix);
		}
		else {
			lcd.print("Track ");
			lcd.print(rssiTrack);
			lcd.print(" Fix ");
			lcd.print(rssiFix);
		}

		lcd.setCursor(0, 1);

		if (isTelemetryOk) {
			lcd.print("Link OK");
		}
		else {
			lcd.print("!! LINK LOST !!");
		}
	}
	else if (trackingMode == RSSITrackingMode) {
		lcd.print("RSSI Track ");
		lcd.print(rssiTrack);
		lcd.setCursor(0, 1);
		lcd.print("RSSI Fix   ");
		lcd.print(rssiFix);
	}
}

void checkSwitchState() {
	if (digitalRead(trackingModeSwitchPin) != lastTrackingModeSwitchState) {
		lcd.clear();

		determineTrackingMode();
	}
	if (trackingMode == GPSTrackingMode && digitalRead(protocolTypeSwitchPin) != lastProtocolTypeSwitchState) {
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

// called at 10Hz
void requestMikrokopterTelemetryData() {
	usart1_EnableTXD();

	// request OSD Data from NC every 100ms
	usart1_puts_pgm(REQUEST_OSD_DATA);

	usart1_DisableTXD();
}

void measureBatteryVoltage() {
      battVoltage = analogRead(battMonitorPin);
      battVoltage = (battVoltage / 1024) * BATT_AREF;
      battVoltage /= (float)BATT_R_LOW / (BATT_R_HIGH + BATT_R_LOW);
      
      if (battVoltage < 6.6) isBattLow = true;
}
