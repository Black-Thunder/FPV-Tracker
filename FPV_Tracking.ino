#include <LiquidCrystal.h>
#include <Servo.h>

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
int serialHorizontalServoOverride = -1;
int serialVerticalServoOverride = -1;

char queryType = 'X';

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

Servo VerticalServo;
Servo HorizontalServo;

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

void setup()
{
    lcd.begin(16, 2);
    VerticalServo.attach(10);
    VerticalServo.write(verticalMid);
    HorizontalServo.attach(11);
    HorizontalServo.write(horizontalMid);
   
    Serial.begin(115200);
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

void loop()
{
    currentTime = micros();
    deltaTime = currentTime - previousTime;

    // ================================================================
    // 100Hz task loop
    // ================================================================
    if (deltaTime >= 10000) {
        frameCounter++;
      
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

void process50HzTask() { }

void processTracking() {
    if(serialHorizontalServoOverride == -1 && serialVerticalServoOverride == -1) {
        readRSSI();
  
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
}

void process10HzTask() {
     readSerialCommand();
}

void process5HzTask() {
     processTracking(); 
}

void process1HzTask() {
    updateLCD();
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

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    queryType = Serial.read();
    switch (queryType) {
    case 'A': 
      serialHorizontalServoOverride = readIntegerSerial();
      serialHorizontalServoOverride = constrain(serialHorizontalServoOverride, horizontalMin, horizontalMax);        
      HorizontalServo.write(serialHorizontalServoOverride);
      break;
    case 'B':
      serialVerticalServoOverride = readIntegerSerial();
      serialVerticalServoOverride = constrain(serialVerticalServoOverride, verticalMin, verticalMax);   
      VerticalServo.write(serialVerticalServoOverride);
      break;
    case 'X':
      HorizontalServo.write(horizontalMid);
      VerticalServo.write(verticalMid);
      serialHorizontalServoOverride = -1;
      serialVerticalServoOverride = -1;
      break;
    }
  }
}

long readIntegerSerial() {
  char data[16] = "";

  readValueSerial(data, sizeof(data));
  return atol(data);
}

void readValueSerial(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (Serial.available() == 0) {
      delay(1);
      timeout++;
    } else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}
