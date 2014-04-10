#ifndef RSSI_TRACKER_H
#define RSSI_TRACKER_H

void calculateRSSIDiff();
void trackHorizontal();
void trackVertical();

extern unsigned char rssiDiv;

#endif
