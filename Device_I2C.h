#ifndef _AEROQUAD_DEVICE_I2C_H_
#define _AEROQUAD_DEVICE_I2C_H_

// I2C functions
#include "Wire.h"

void sendByteI2C(int deviceAddress, uint8_t dataValue);
uint8_t readByteI2C();
uint8_t readByteI2C(int deviceAddress);
int readWordI2C(int deviceAddress);
int readWordI2C();
int readShortI2C(int deviceAddress);
int readShortI2C();
int readReverseShortI2C();
int readWordWaitI2C(int deviceAddress);
int readReverseWordI2C(int deviceAddress);
uint8_t readWhoI2C(int deviceAddress);
void updateRegisterI2C(int deviceAddress, uint8_t dataAddress, uint8_t dataValue);

#endif





