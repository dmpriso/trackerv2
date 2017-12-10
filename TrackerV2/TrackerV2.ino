#include <quaternionFilters.h>
#include <MPU9250.h>
#include <timer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#include "motor.h"
#include "mag_calib.h"

Timer tmrMotor;
MPU9250 imu;
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
MagCalib mag;

float fHardCalibDegreeCounter = 360.0f * 5.f;
bool bHardCalibDone = false;
float fSoftCalibDegreeCounter = 360.0f * 5.f;
bool bSoftCalibDone = false;
float fMagOffset;

void onMotorPosition(float iPosition);

TrackerMotor motor(10, 11, 300, [] (int iAfterMs) {
	if (0 != iAfterMs)
	{
		tmrMotor.after(iAfterMs, [] { motor.onTimerElapsed(); });
	}
}, [](float fPosition)
{
	onMotorPosition(fPosition);
});

void error(char* txt, int num = -32768) {
	display.clearDisplay();
	display.setTextSize(2);
	display.setCursor(0, 0);
	display.println("Error");
	display.setTextSize(1);
	display.println(txt);
	if (num != -32768)
		display.print(num);
	display.display();

	while (true) { ; }
}

void initMPU() {
	Wire.begin();

	// check MPU
	// Read the WHO_AM_I register, this is a good test of communication
	byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if (c != 0x71) {
		error("Could not find MPU");
	}

	// init MPU
	imu.initMPU9250();

	// check compass
	byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	if (d != 0x48) {
		error("Could not find MAG", d);
	}

	// read magnetometer calibration
	imu.initAK8963(imu.magCalibration);


}

void setup() {
	display.begin();
	display.setContrast(58); // Set the contrast
	display.clearDisplay();
	display.print("Initialization...");
	display.display();
	delay(1000);

	Serial.begin(115200);
	while (!Serial) { ; }

	initMPU();
	
	display.println("OK");
	display.print("Calibration ...");
	display.display();
	//delay(1000);

	motor.setContinuousSpeed(200);
}

bool readMagnetometer()
{
	imu.readMagData(imu.magCount);  // Read the x/y/z adc values
	imu.getMres();

	static float lastX = 0.f, lastY = 0.f, lastZ = 0.f;

	imu.mx = (float)imu.magCount[0] * imu.mRes*imu.magCalibration[0] -
		imu.magbias[0];
	imu.my = (float)imu.magCount[1] * imu.mRes*imu.magCalibration[1] -
		imu.magbias[1];
	imu.mz = (float)imu.magCount[2] * imu.mRes*imu.magCalibration[2] -
		imu.magbias[2];

	if (lastX != imu.mx || lastY != imu.my || lastZ != imu.mz)
	{
		lastX = imu.mx; lastY = imu.my, lastZ = imu.mz;
		return true;
	}
	return false;

	/*
	auto printAxis = [](const char* axis, float v)
	{
		Serial.print("Axis ");
		Serial.print(axis);
		Serial.print(": ");
		Serial.println(v);
	};

	printAxis("x", imu.mx);
	printAxis("y", imu.my);
	Serial.println();
	//printAxis("z", imu.mz);
	*/
}

void onMotorPosition(float fPosition)
{
	static float fLastPosition = 0.0f;

	float fMove = abs(fPosition - fLastPosition);
	fLastPosition = fPosition;

	if (fHardCalibDegreeCounter > 0.f)
	{
		if (readMagnetometer())
			mag.putCenterCalibValue(imu.mx, imu.my);
		fHardCalibDegreeCounter -= fMove;
	}
	else if (fSoftCalibDegreeCounter > 0.f)
	{
		if (!bHardCalibDone)
		{
			bHardCalibDone = true;
			mag.finishCenterCalib();
		}

		if (readMagnetometer())
			mag.putCalibValue(fPosition, imu.mx, imu.my);
		fSoftCalibDegreeCounter -= fMove;
	}
	else if (!bSoftCalibDone)
	{
		bSoftCalibDone = true;
		fMagOffset = mag.finishCalib();

		display.println("OK");
		display.display();

		motor.setContinuousSpeed(0);
		motor.setPosition((int)-fMagOffset);
	}
}

void loop()
{
	tmrMotor.update();
}

