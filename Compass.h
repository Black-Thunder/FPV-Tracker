#ifndef _AEROQUAD_COMPASS_H_
#define _AEROQUAD_COMPASS_H_

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)

#define HMC5883L

float hdgX = 0.0;
float hdgY = 0.0;

float measuredMagX = 0.0;
float measuredMagY = 0.0;
float measuredMagZ = 0.0;
float measuredMag[3] = {0.0,0.0,0.0};
float rawMag[3] = {0.0,0.0,0.0};
float magBias[3] = {0.0,0.0,0.0};

void initializeMagnetometer();
void measureMagnetometer(float roll, float pitch);

const float getHdgXY(uint8_t axis) {
  if (axis == XAXIS) {
    return hdgX;
  } else {
    return hdgY;
  }
}

const int getMagnetometerRawData(uint8_t axis) {
  return rawMag[axis];
}

const int getMagnetometerData(uint8_t axis) {
  return measuredMag[axis];
}


const float getAbsoluteHeading() {
  float heading = atan2(hdgY, hdgX);
  if (heading < 0) {
	heading += radians(360);
  }
  return heading;
}

#endif
