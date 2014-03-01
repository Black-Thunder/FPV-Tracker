#include <LiquidCrystal.h>
#include <Servo.h>

int verticalTolerance = 3;
int horizontallTolerance = 2;
int thresholdValue = 85;
int verticalMid = 75;
int horizontalMid = 90;
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
#define TASK_1HZ 100

#define NUMBER_OF_SAMPLES 10

Servo VerticalServo;
Servo HorizontalServo;

int rssi1 = A0;
int rssi2 = A1;

int rssiTrack = 0;
int rssiFix = 0;
int rssiTrackOld = 0;
int rssiDiv = 0;
int i = horizontalMid;
int y = 0;
char horizontalDirection;
char verticalDirection;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup()
{
    lcd.begin(16, 2);
    VerticalServo.attach(10);  // attaches the servo on pin 10 to the servo object
    VerticalServo.write(verticalMid);
    HorizontalServo.attach(11);  // attaches the servo on pin 11 to the servo object
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
      process100HzTask();
      frameCounter++;
      
    if(frameCounter % TASK_50HZ == 0) {
      process50HzTask();  
    }
      
      previousTime = currentTime;
    }
    
    if(frameCounter % TASK_10HZ == 0) {   //   10 Hz tasks
      process10HzTask();
    }
    
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
  
    if (frameCounter >= 100) {
      frameCounter = 0;
    }
}

void process100HzTask(){}

void process50HzTask() {
    if(serialHorizontalServoOverride == -1 && serialVerticalServoOverride == -1) {
        readRSSI();
  
        if(rssiTrack <= thresholdValue)
        {
            trackHorizontal();
        }
    }
    else {
        if(serialHorizontalServoOverride != -1) HorizontalServo.write(serialHorizontalServoOverride);
        if(serialVerticalServoOverride != -1) VerticalServo.write(serialVerticalServoOverride);
    }
}

void process10HzTask() {
     readSerialCommand();
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
    // Map values to defined range
    rssiTrack = map(analogRead(rssi1), 0, calibrate1, 0, 100);
    rssiFix = map(analogRead(rssi2), 0, calibrate2, 0, 100);
    
    if (rssiTrack>100) {
        rssiTrack = 100;
    }
    if (rssiFix>100) {
        rssiFix = 100;
    }
}

void trackHorizontal()
{    
    //i = horizontalMid;
    
    //do {
      //  readRSSI();
            
        rssiDiv = (rssiTrack-rssiTrackOld);
        
        if (rssiDiv < 0) {
          rssiDiv = rssiDiv * -1;
        }
        
        if(rssiDiv <= horizontallTolerance ) {   
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
                trackVertikal();
            }
        }
        else {
            if(rssiTrack>rssiTrackOld) {
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
        }
    
        HorizontalServo.write(i);
    
        if (i <= 0 || i >= 180) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Reset");
        
            i = horizontalMid;
            HorizontalServo.write(horizontalMid);
            VerticalServo.write(verticalMid);
            return;
         }
    //}
    //while (rssiTrack <= 98);
}


void trackVertikal() {
    int loopi = 0;
    
    do {
        readRSSI();
        
        rssiDiv = (rssiTrack-rssiTrackOld);
        
        if (rssiDiv < 0) {
            rssiDiv = rssiDiv * -1;
        }
        
        if( rssiDiv <= horizontallTolerance ) {   
            if(rssiTrack <= 45) {
               y = verticalMid;
               VerticalServo.write(y);
               return;               
            }
        }
        else {
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
        }
 
        VerticalServo.write(y);
        
        if (y <= 10 || y >= 100) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Reset");
            
            y=verticalMid;
            return;
        }
        
        loopi++;
        }
    while (loopi <= 4);
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
      break;
    case 'B':
      serialVerticalServoOverride = readIntegerSerial();
      break;
    case 'X':
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
