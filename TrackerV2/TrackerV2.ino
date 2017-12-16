#include <MicroNMEA.h>
#include <quaternionFilters.h>
#include <Timer-master\Timer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Servo.h>

#include "MPU_Small.h"
#include "motor.h"
#include "mag_calib.h"
#include "bluetooth.h"
#include "stream_processor.h"
#include "limited_string.h"
#include "mavlink_processor.h"
#include "buzzer.h"

#define BT_CONNECT_PIN 12
#define BT_BINDSWITCH_PIN 11
#define MOTOR_DIR_PIN 4
#define MOTOR_STEP_PIN 3
#define MOTOR_FAULT_PIN 2
#define TILT_SERVO_PIN 5
#define TILT_SERVO_HORIZONTAL_PWM 1930
#define TILT_SERVO_VERTIAL_PWM 1060
#define LCD_SCLK_PIN 13
#define LCD_DIN_PIN 14
#define LCD_DC_PIN 15
#define LCD_CS_PIN 22
#define LCD_RST_PIN 20
#define BUZZER_PIN 23
#define VSENS_AREF 3.3f
#define VSENS_APIN 3
#define VSENS_FACTOR 6.0f
#define COMPASS_ROTATION 90.0f


Timer tmrMotor;
MPU9250_CompassOnly imu;
Adafruit_PCD8544 display = Adafruit_PCD8544(LCD_SCLK_PIN, LCD_DIN_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_RST_PIN);
MagCalib mag;
Bluetooth xfire(Serial3, BT_CONNECT_PIN, BT_BINDSWITCH_PIN, true, showBTStatus, showBTError, showBTConnectStatus, selectBTPeer, processBTData);
MavlinkProcessor mavlink;
HardwareSerial& gps = Serial2;
char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));
Buzzer buzzer(BUZZER_PIN);
Servo tilt;

float fHardCalibDegreeCounter = 360.0f * 5.f;
bool bHardCalibDone = false;
float fSoftCalibDegreeCounter = 360.0f * 5.f;
bool bSoftCalibDone = false;
float fMagOffset;

struct SystemStatus
{
	bool bMagCalibrationDone = false;
	float vbat = 0.0f;
	float vbat_highest = 0.0f;
	float vbat_per_cell = 0.0f;
	int bt_status = 0;
	bool mavlink_active = false;
	bool gps_has_fix = false;

	float gps_lon = 0.0f;
	float gps_lat = 0.0f;
	float gps_alt = 0.0f;

	inline bool operator==(const SystemStatus& comp) const
	{
		return (bMagCalibrationDone == comp.bMagCalibrationDone &&
			vbat == comp.vbat &&
			vbat_highest == comp.vbat_highest &&
			vbat_per_cell == comp.vbat_per_cell &&
			bt_status == comp.bt_status &&
			mavlink_active == comp.mavlink_active &&
			gps_has_fix == comp.gps_has_fix);
	}
	inline bool operator!=(const SystemStatus& comp) const { return !this->operator==(comp); }
};
SystemStatus status;

void onMotorPosition(float iPosition);

TrackerMotor motor(MOTOR_STEP_PIN, MOTOR_DIR_PIN, 300, [] (int iAfterMs) {
	if (0 != iAfterMs)
	{
		tmrMotor.after(iAfterMs, [] { motor.onTimerElapsed(); });
	}
}, [](float fPosition)
{
	onMotorPosition(fPosition);
});

void error(const __FlashStringHelper* txt, int num = -32768) {
	display.clearDisplay();
	display.setTextSize(2);
	display.setCursor(0, 0);
	display.println(F("Error"));
	display.setTextSize(1);
	display.println(txt);
	if (num != -32768)
		display.print(num);
	display.display();

	while (true) { ; }
}

void I2Cscan()
{
	// scan for i2c devices
	byte error, address;
	int nDevices;
	Serial.println("Scanning...");
	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");
			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
}

void initMPU() {
	Wire.begin();

	delay(1500);
	I2Cscan();

	/*
	// check MPU
	// Read the WHO_AM_I register, this is a good test of communication
	byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	Serial.println(c);
	if (c != 0x71) {
		error(F("Could not find MPU"));
	}
	
	delay(1000);*/

	// set MPU to bypass mode
	imu.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);

	// check compass
	byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	Serial.println(d);

	if (d != 0x48) {
		error(F("Could not find magnetometer"), d);
	}
	
	// read magnetometer calibration
	imu.initAK8963(imu.magCalibration);
}

void showStatus()
{
	static SystemStatus laststatus;

	if (laststatus != status)
	{
		laststatus = status;

		display.clearDisplay();
		display.setTextSize(2);
		display.setCursor(0, 0);

		if (!status.bMagCalibrationDone)
		{
			display.print(F("Compass"));
			display.setTextSize(1);
			display.setCursor(0, 16);
			display.print(F("Performing compass calibration. Please wait ..."));
		}
		else
		{
			display.print(F("Status"));

			display.setTextSize(1);

			// battery
			display.setCursor(0, 16);
			display.print(F("Battery: "));
			display.print(status.vbat, 1);
			display.print(F("V"));

			// bluetooth
			display.setCursor(0, 24);
			display.print(F("BT: "));
			if (status.bt_status == 2)
				display.print(F("Searching"));
			else if (status.bt_status == 1)
				display.print(F("OK"));
			else
				display.print(F("Connecting"));

			// mavlink
			display.setCursor(0, 32);
			display.print(F("Tlmtry: "));
			if (status.mavlink_active)
				display.print(F("OK"));
			else
				display.print(F("NOK"));

			// gps fix
			display.setCursor(0, 40);
			display.print(F("GPS: "));
			if (status.gps_has_fix)
				display.print(F("OK"));
			else
				display.print(F("Acquiring"));
		}
		display.display();
	}
}

// displays a bluetooth connect error
void showBTError(const __FlashStringHelper* err) {
	//showStatus(F("XFireBT"), F("Error"), err);
	Serial.print(F("BTError: "));
	Serial.println(err);

	// we would need to wait for user input here
	// instead we simply delay for 5 seconds
}

void showBTStatus(const __FlashStringHelper* stat) {
	//showStatus(F("XFireBT"), stat);
	Serial.print(F("BTStatus: "));
	Serial.println(stat);

}

void showBTConnectStatus(int connected) {
	status.bt_status = connected;
}

bool selectBTPeer(const char* peername) {
	// parse all lines
	ConstString str(peername);
	int id = str.indexOf('\n');
	int start = 0;

	Serial.println(id);

	while (-1 != id)
	{
		int id1 = str.indexOf(',', start);


		if (-1 != id1)
		{
			// first part is MAC
			// second part is name
			ConstString strName(peername + id1 + 1, id - id1 - 1);

			if (strName.startsWith(F("Crossfire")))
			{
				// we got a x-fire device!
				// bind it
				ConstString strMac(peername + start, id1 - start);
				xfire.setPeer(strMac);
				return true;
			}
		}

		start = id + 1;
		id = str.indexOf('\n', start);
	}

	return false;
}

void processBTData(char chr)
{
	mavlink.process(chr);
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
}

void setPanDegrees(float fDegrees)
{
	motor.setPosition((int)(fDegrees - fMagOffset - COMPASS_ROTATION));
}

void setTiltDegrees(int degrees)
{
	degrees = min(max(degrees, 0), 90);

	tilt.writeMicroseconds(TILT_SERVO_HORIZONTAL_PWM +
		(TILT_SERVO_VERTIAL_PWM - TILT_SERVO_HORIZONTAL_PWM) * degrees / 90);
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

		motor.setContinuousSpeed(0);
		setPanDegrees(0);

		status.bMagCalibrationDone = true;
	}
}

void setup() {
	// bind switch
	//pinMode(BT_BINDSWITCH_PIN, INPUT_PULLUP);

	Serial.begin(9600);
	gps.begin(9600);

	display.begin();
	display.setContrast(58); // Set the contrast
	display.clearDisplay();

	tilt.attach(TILT_SERVO_PIN);
	setTiltDegrees(45);

	initMPU();

	showStatus();

	// alert beeping
	buzzer.setBuzzing(1000, 1000, 3);

	// start with compass calibration after 5 seconds
	tmrMotor.after(5000, [] {
		motor.setContinuousSpeed(200);
	});
}

void loop()
{
	// buzzer
	buzzer.update();

	// read vbat
	static uint32_t next_vbat_read = 0;
	if (millis() >= next_vbat_read)
	{
		status.vbat = (float)analogRead(VSENS_APIN) / 1024.0f * VSENS_AREF * VSENS_FACTOR;
		status.vbat_highest = max(status.vbat, status.vbat_highest);
		// detect number of cells depending on vbat_highest
		float numcells = (status.vbat_highest > 13.0f ? 4.0f : 3.0f);
		status.vbat_per_cell = status.vbat / numcells;
		next_vbat_read = millis() + 2000;
	}

	// motor
	tmrMotor.update();
	
	// let bluetooth/mavlink do its work
	xfire.onLoop();

	// process GPS
	if (gps.available()) {
		char c = gps.read();
		if (nmea.process(c))
		{
			if (nmea.isValid())
			{
				status.gps_lat = (float)nmea.getLatitude() / 1000000.0f;
				status.gps_lon = (float)nmea.getLongitude() / 1000000.0f;
				long alt;
				if (nmea.getAltitude(alt))
					status.gps_alt = (float)alt / 1000.0f;
				status.gps_has_fix = true;
			}
		}
	}

	// read mavlink
	float lat;
	float lon;
	float alt;

	static bool bFullFix = false;

	if ((status.mavlink_active = mavlink.getPos(lat, lon, alt)) && status.gps_has_fix)
	{
		if (!bFullFix)
		{
			bFullFix = true;
			buzzer.setBuzzing(20, 3000);
		}

		Serial.print("GPS lat: "); Serial.print(status.gps_lat); Serial.print(" MAV lat: "); Serial.println(lat);
		Serial.print("GPS lon: "); Serial.print(status.gps_lon); Serial.print(" MAV lon: "); Serial.println(lon);
		Serial.print("GPS alt: "); Serial.print(status.gps_alt); Serial.print(" MAV alt: "); Serial.println(alt);
		Serial.println("");
	}
	else if (bFullFix)
	{
		bFullFix = false;

		// alert buzzing: we lost mavlink
		buzzer.setBuzzing(1000, 500, 10);
	}

	showStatus();
}

