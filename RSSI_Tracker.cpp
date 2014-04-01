#include "RSSI_Tracker.h"
#include "TrackerGCS.h"

int rssiDiv = 0;

void calculateRSSIDiff() {
	rssiDiv = rssiTrack - rssiTrackOld;

	if (rssiDiv < 0) {
		rssiDiv *= -1;
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

	if (i <= horizontalMin || i >= horizontalMax) {
		i = horizontalMid;
		applyServoCommand(horizontalServo, horizontalMid);
		applyServoCommand(verticalServo, verticalMid);
		return;
	}

	applyServoCommand(horizontalServo, i);
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

	if (y <= verticalMin || y >= verticalMax) {
		y = verticalMid;
		applyServoCommand(horizontalServo, horizontalMid);
		applyServoCommand(verticalServo, verticalMid);
		return;
	}

	applyServoCommand(verticalServo, y);
}