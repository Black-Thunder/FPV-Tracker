#include "config.h"
#include <Servo.h>
#include "TrackerGCS.h"

#if defined PRO_MINI
#define RSSI_TRACKING

unsigned char trackingMode = RSSITrackingMode;
#endif

#if defined MEGA
#define GPS_TRACKING
#define RSSI_TRACKING
#endif

#if defined LCD_AVAILABLE
#include <LiquidCrystal.h>
#endif

#if defined RSSI_TRACKING
#include "RSSI_Tracker.h"
#endif

#if defined GPS_TRACKING
#include "GPS_Tracker.h"
#include "HMC5883L.h"
#include "uart1.h"
#include "Mikrokopter_Datastructs.h"
#include "AeroQuad_Datastructs.h"
#endif

// RSSI tracking
#if defined RSSI_TRACKING
const unsigned char horizontalTolerance = 2;
const unsigned char thresholdValue = 80;

const unsigned char memorySize = 15;
const unsigned char rssiIndex = 0;
const unsigned char verticalIndex = 1;
const unsigned char horizontalIndex = 2;
unsigned char rssiTrackingVariablesMemory[memorySize][horizontalIndex + 1];
unsigned char rssiTrackingCounter = 0;
bool isRSSITrackingStopped = false;

int calibrateTrack = 0;
int calibrateFix = 0;
unsigned char i = horizontalMid;
unsigned char y = verticalMid;

unsigned char horizontalDirection = 0;
unsigned char verticalDirection = 0;
#endif

// GPS tracking
#if defined GPS_TRACKING
unsigned char protocolType = AeroQuadProtocol;
const unsigned char protocolTypeSwitchPin = 8; // Hardware switch to determine protocol type; LOW=AeroQuad, HIGH=Mikrokopter
bool lastProtocolTypeSwitchState = LOW;

float uavLatitude = GPS_INVALID_ANGLE;
float uavLongitude = GPS_INVALID_ANGLE;
unsigned char uavSatellitesVisible = 0;
int16_t uavAltitude = GPS_INVALID_ALTITUDE;

float homeLongitude = GPS_INVALID_ANGLE;
float homeLatitude = GPS_INVALID_ANGLE;
float uavDistanceToHome = 0;
unsigned int homeBearing = 0;

unsigned int trackingBearing = 0;
unsigned int trackingElevation = 0;

bool uavHasGPSFix = false;
bool isTelemetryOk = false;
long lastPacketReceived = 0;
#endif

// General
#if !defined PRO_MINI
unsigned char trackingMode = GPSTrackingMode;
const unsigned char trackingModeSwitchPin = 9; // Hardware switch to determine tracking mode; LOW=RSSI, HIGH=GPS
bool lastTrackingModeSwitchState = LOW;
#endif

#if defined LCD_AVAILABLE
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

byte okSmiley[8] = {
	B00000,
	B10001,
	B00000,
	B00100,
	B00000,
	B10001,
	B01110,
	B00000
};

byte badSmiley[8] = {
	B00000,
	B10001,
	B00000,
	B00100,
	B00000,
	B01110,
	B10001,
	B00000
};
#endif

unsigned char rssiTrack = 0;
unsigned char rssiFix = 0;
unsigned char rssiTrackOld = 0;

unsigned char servoCommands[2] = { verticalMid, horizontalMid };
unsigned char previousServoCommands[2] = { -1, -1 };

Servo VerticalServo;
Servo HorizontalServo;

float battVoltage = 0;
bool isBattLow = false;

// Main loop time variables
unsigned char frameCounter = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100

void calibrateRSSI() {
#if defined LCD_AVAILABLE
	lcd.setCursor(0, 2);
	lcd.print("Calibrating RSSI... ");
#endif

	for (unsigned char counter = 0; counter < numberOfRSSISamples; counter++) {
		calibrateTrack += analogRead(rssiTrackPin);
		delay(50);
	}
	calibrateTrack /= numberOfRSSISamples;

#if defined DIVERSITY
	for (unsigned char counter = 0; counter < numberOfRSSISamples; counter++) {
		calibrateFix += analogRead(rssiFixPin);
		delay(50);
	}
	calibrateFix /= numberOfRSSISamples;
#endif
}

#if !defined PRO_MINI
void determineProtocolType() {
	if (digitalRead(protocolTypeSwitchPin) == HIGH) {
		protocolType = MikrokopterProtocol;
		lastProtocolTypeSwitchState = HIGH;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: MK        ");
	}
	else {
		protocolType = AeroQuadProtocol;
		lastProtocolTypeSwitchState = LOW;

		lcd.setCursor(0, 1);
		lcd.print("Protocol: AQ        ");
	}
}

void setupUAVCommunication() {
	if (protocolType == MikrokopterProtocol) {
		// request NC uart from MK, already done by C-OSD/Smart-OSD
		//usart1_request_nc_uart();
	}
}

void setupHMC5883L(){
	compass = HMC5883L();
	compass.CheckConnectionState();

	lcd.setCursor(0, 2);
	lcd.print("Configuring Mag...  ");

	if (compass.isMagDetected) {
		compass.SetScale(1.3); //Set the scale of the compass.
		compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	}
	else {
		lcd.setCursor(0, 2);
		lcd.print("Mag Failure!        ");
		lcd.setCursor(0, 3);
		lcd.print("Heading set to 0    ");

		delay(2000);  // Keep LCD message visible
	}
}

void setupGPSTrackingMode() {
	determineProtocolType();

	usart1_init();
	usart1_DisableTXD();
	sei();

	setupUAVCommunication();

	lcd.setCursor(0, 2);
	lcd.print("Configuring GPS...  ");
	initializeGps();

	if (!isGPSConfigured) {
		lcd.setCursor(0, 2);
		lcd.print("GPS Failure!        ");
		lcd.setCursor(0, 3);
		lcd.print("No Tracking!        ");
		delay(2000);  // Keep LCD message visible
	}

	setupHMC5883L();
}

void determineTrackingMode() {
	if (digitalRead(trackingModeSwitchPin) == HIGH) {
		trackingMode = RSSITrackingMode;
		lastTrackingModeSwitchState = HIGH;

		lcd.setCursor(0, 0);
		lcd.print("Mode: RSSI          ");
	}
	else {
		trackingMode = GPSTrackingMode;
		lastTrackingModeSwitchState = LOW;

		lcd.setCursor(0, 0);
		lcd.print("Mode: GPS           ");

		setupGPSTrackingMode();
	}
}
#endif

void processTracking() {
	if (trackingMode == RSSITrackingMode) {
		if (!isRSSITrackingStopped && rssiTrackingCounter >= memorySize) {
			// Find highest RSSI value within the past 5 seconds and move servos to the corresponding position
			unsigned char maxValueIndex = 0;
			unsigned char maxValue = 0;

			for (unsigned char i = 0; i < memorySize; i++) {
				if (rssiTrackingVariablesMemory[i][rssiIndex] > maxValue) {
					maxValue = rssiTrackingVariablesMemory[i][rssiIndex];
					maxValueIndex = i;
				}
			}

			applyServoCommand(verticalServo, rssiTrackingVariablesMemory[maxValueIndex][verticalIndex]);
			applyServoCommand(horizontalServo, rssiTrackingVariablesMemory[maxValueIndex][horizontalIndex]);
			isRSSITrackingStopped = true;
		}
		else if (!isRSSITrackingStopped && rssiTrack <= thresholdValue) {
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

			// Store current RSSI value and the corresponding servo positions
			rssiTrackingVariablesMemory[rssiTrackingCounter][rssiIndex] = rssiTrack;
			rssiTrackingVariablesMemory[rssiTrackingCounter][verticalIndex] = servoCommands[verticalServo];
			rssiTrackingVariablesMemory[rssiTrackingCounter][horizontalIndex] = servoCommands[horizontalServo];
			rssiTrackingCounter++;
		}
		else if (rssiTrack > thresholdValue) {
			// Reset rssiTrackingCounter and start tracking
			rssiTrackingCounter = 0;
			isRSSITrackingStopped = false;
		}
	}
#if !defined PRO_MINI
	if (trackingMode == GPSTrackingMode) {
		// Only move servo if home position is set, data link is OK and UAV has GPS fix, otherwise standby to last known position
		if (isHomeBaseInitialized() && isTelemetryOk && uavHasGPSFix) {
			calculateTrackingVariables(homeLongitude, homeLatitude, uavLongitude, uavLatitude, uavAltitude);

			if (uavDistanceToHome > minTrackingDistance) {
				// Set current GPS bearing relative to homeBearing
				if (trackingBearing >= homeBearing) {
					trackingBearing -= homeBearing;
				}
				else {
					trackingBearing += 360 - homeBearing;
				}

				/* Map tracking variables to servo range
				* trackingBearing: 0 = North, 90 = West, 180 = South, 270 = East
				* trackingElevation: 0 = parallel to ground, 90 = straight up into the sky
				* Servo angle: 0 = Maximum left/down, 90 = Mid, 180 = Maximum right/up
				*/
				if (trackingBearing >= 0 && trackingBearing <= 90) {
					trackingBearing = map(trackingBearing, 90, 0, horizontalMin, horizontalMid);
				}
				else if (trackingBearing >= 270 && trackingBearing <= 360) {
					trackingBearing = map(trackingBearing, 360, 270, horizontalMid, horizontalMax);
				}
				else if (trackingBearing > 90 && trackingBearing < 180) {
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
#endif
}

void applyServoCommand(int servo, unsigned int value) {
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

#if defined LCD_AVAILABLE
void printVoltage() {
#if defined SMALL_LCD
        lcd.setCursor(0, 2);
#else 
	lcd.setCursor(0, 3);
	lcd.print("Voltage: ");
#endif
	lcd.print(battVoltage);
	lcd.print("V");

	if (isBattLow) {
		lcd.print(" LOW !!");
	}
}

void updateLCD() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("RSSI Track ");
	lcd.print(rssiTrack);
#if defined DIVERSITY
	lcd.setCursor(0, 1);
	lcd.print("RSSI Fix   ");
	lcd.print(rssiFix);
#endif
#if defined GPS_TRACKING
	if (trackingMode == GPSTrackingMode) {
		lcd.setCursor(0, 2);

		if (isTelemetryOk) {
			lcd.print("Link OK ");
			lcd.write(byte(0));
		}
		else {
			lcd.print("!! LINK LOST !! ");
			lcd.write(1);
		}

		if (!isGPSConfigured) {
			lcd.setCursor(0, 3);
			lcd.print("GPS Failure!       ");
		}
		else if (!isHomeBaseInitialized()) {
			lcd.setCursor(0, 3);
			lcd.print("Waiting for GPS fix");
		}
		else {
			printVoltage();
		}
	}
#endif	
        if (trackingMode == RSSITrackingMode) {
		printVoltage();
	}
}

void measureBatteryVoltage() {
	battVoltage = analogRead(battMonitorPin);
	battVoltage = (battVoltage * battAREFValue) / 1024.0;
	battVoltage /= (battResistorLow / (battResistorHigh + battResistorLow));

	if (battVoltage < 6.6) isBattLow = true;
	else isBattLow = false;
}
#endif

#if !defined PRO_MINI
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

void requestMikrokopterTelemetryData() {
	usart1_EnableTXD();

	// request OSD Data from NC every 100ms
	usart1_puts_pgm(REQUEST_OSD_DATA);

	usart1_DisableTXD();
}
#endif

void readRSSI() {
	rssiTrackOld = rssiTrack;

	rssiTrack = map(analogRead(rssiTrackPin), 0, calibrateTrack, 0, 100);
	rssiTrack = constrain(rssiTrack, 0, 100);

#if defined DIVERSITY
	rssiFix = map(analogRead(rssiFixPin), 0, calibrateFix, 0, 100);
	rssiFix = constrain(rssiFix, 0, 100);
#endif
}

void process100HzTask() {
#if !defined PRO_MINI
	if (trackingMode == GPSTrackingMode) {
		updateGps();
	}
#endif
}

void process50HzTask() {
	writeServos();
}

void process10HzTask() {
#if !defined PRO_MINI
	if (trackingMode == GPSTrackingMode) {
		// Request data from MK, this is already done by C-OSD/Smart-OSD
		//if (protocolType == MikrokopterProtocol) {
		//	requestMikrokopterTelemetryData();
		//}

		processUsart1Data();

		// No telemetry data received for more than 2.5 seconds
		if (millis() - lastPacketReceived > 2500) {
			isTelemetryOk = false;
		}
	}
#endif
}

void process5HzTask() {
#if !defined PRO_MINI
	if (trackingMode == GPSTrackingMode) {
		if (!isHomeBaseInitialized() && isGPSConfigured) {
			updateGCSPosition();
		}

		if (compass.isMagDetected) {
			updateGCSHeading();
		}
	}
#endif

	readRSSI();
	processTracking();
}

void process1HzTask() {
#if defined LCD_AVAILABLE
	measureBatteryVoltage();
	updateLCD();
#endif

#if !defined PRO_MINI
	checkSwitchState();
#endif
}

// ================================================================
// Setup section
// ================================================================

void setup() {
#if defined LCD_AVAILABLE
#if defined SMALL_LCD
        lcd.begin(16, 2);
#else
	lcd.begin(20, 4);
	lcd.createChar(0, okSmiley);
	lcd.createChar(1, badSmiley);
#endif
#endif

	VerticalServo.attach(verticalServoPin);
	HorizontalServo.attach(horizontalServoPin);
	writeServos(); // move servos to middle position on start-up

#if !defined PRO_MINI
	determineTrackingMode();
#endif

	calibrateRSSI();
}

// ================================================================
// Main loop
// ================================================================

void loop() {
	currentTime = micros();
	deltaTime = currentTime - previousTime;

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) {
		frameCounter++;

		process100HzTask();

		// ================================================================
		// 50Hz task loop
		// ================================================================
		if (frameCounter % TASK_50HZ == 0) { // 50 Hz tasks
			process50HzTask();
		}

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
