#include "RSSI_Tracker.h"
#include "TrackerGCS.h"

int rssiDiv = 0;

void calculateRSSIDiff() {
	rssiDiv = (rssiTrack - rssiTrackOld);

	if (rssiDiv < 0) {
		rssiDiv = rssiDiv * -1;
	}
}

void trackHorizontal() {
	if (rssiTrack > rssiTrackOld) {
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
	if (rssiTrack > rssiTrackOld) {
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