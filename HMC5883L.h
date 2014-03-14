#ifndef HMC5883L_h
#define HMC5883L_h

#include <stdint.h>

#define HMC5883L_Address 0x1E
#define COMPASS_IDENTITY 0x10
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

struct MagnetometerScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct MagnetometerRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
};

class HMC5883L
{
public:
	HMC5883L();

	void CheckConnectionState();

	MagnetometerRaw ReadRawAxis();
	MagnetometerScaled ReadScaledAxis();

	void SetMeasurementMode(uint8_t mode);
	void SetScale(float gauss);

	bool isMagDetected;

protected:
	void Write(int address, int byte);
	uint8_t* Read(int address, int length);

private:
	float m_Scale;
};
#endif