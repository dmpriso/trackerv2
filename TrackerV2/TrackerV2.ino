#include <MicroNMEA.h>
#include <quaternionFilters.h>
#include <Timer-master\Timer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Servo.h>

#include "MPU_Small.h"
#include "motor.h"
#include "motor_with_speed_ctrl.h"
#include "rotor_with_pos_ctrl.h"
#include "mag_calib.h"
#include "bluetooth.h"
#include "stream_processor.h"
#include "limited_string.h"
#include "mavlink_processor.h"
#include "buzzer.h"
#include "pos_calc.h"

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

#define PWM_PIN 3
#define DIR_PIN 2
#define ENC_PIN_1 6
#define ENC_PIN_2 4
#define ENC_FACTOR (3591.84 * 3. / 2.)

#define P_SPEED 4.
#define I_SPEED 2.
#define D_SPEED .2

#define P_POS 1.4
#define I_POS 0.01
#define D_POS .001

void onMotorPosition(float iPosition);

Motor motor(PWM_PIN, DIR_PIN, ENC_PIN_1, ENC_PIN_2);
MotorWithSpeedCtrl smotor(motor, P_SPEED, I_SPEED, D_SPEED);
RotorWithPosCtrl rotor(smotor, P_POS, I_POS, D_POS, ENC_FACTOR, [](double pos)
{
	onMotorPosition(pos);
});

IntervalTimer tmr;
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

float fCalibDegreeCounter = 360.0f * 5.f;
float fMagOffset;
float fMagDeclination = 0.f;

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

void initMPU() {
	Wire.begin();

	delay(1500);

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
	rotor.setPosition((int)(fDegrees - fMagOffset - COMPASS_ROTATION - fMagDeclination));
}

void setTiltDegrees(int degrees)
{
	degrees = min(max(degrees, 0), 90);

	auto tmp = TILT_SERVO_HORIZONTAL_PWM +
		(TILT_SERVO_VERTIAL_PWM - TILT_SERVO_HORIZONTAL_PWM) * degrees / 90;
	tilt.writeMicroseconds(tmp);
}

void onMotorPosition(float fPosition)
{
	static float fLastPosition = 0.0f;

	float fMove = abs(fPosition - fLastPosition);
	if (fMove > 180.f)
		fMove = 360.f - fMove;

	fLastPosition = fPosition;

	if (fCalibDegreeCounter > 0.f)
	{
		if (readMagnetometer())
			mag.putCalibValue(fPosition, imu.mx, imu.my);
		fCalibDegreeCounter -= fMove;
	}
	else if (!status.bMagCalibrationDone)
	{
		fMagOffset = mag.finishCalib();

		rotor.setContinuousSpeed(0);
		setPanDegrees(0);

		status.bMagCalibrationDone = true;
	}
}

void setup() {
	// bind switch
	//pinMode(BT_BINDSWITCH_PIN, INPUT_PULLUP);

	Serial.begin(115200);
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

	// 10ms timer for rotor
	tmr.begin([]
	{
		rotor.on10000usElapsed();
	}, 10000.);

	// start with compass calibration after 5 seconds
	tmrMotor.after(5000, [] {
		rotor.setContinuousSpeed(180);
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
	rotor.update();
	
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

				fMagDeclination = PosCalc::getMagDeclination(status.gps_lat, status.gps_lon);
				Serial.print("MAG Declination: ");
				Serial.println(fMagDeclination);
			}
		}
	}

	// read mavlink
	float lat;
	float lon;
	float alt;

	static bool bFullFix = false;

	if (digitalRead(BT_BINDSWITCH_PIN) == LOW)
	{
		setPanDegrees(-100);
	}

	if ((status.mavlink_active = mavlink.getPos(lat, lon, alt)) && status.gps_has_fix)
	{
		if (!bFullFix)
		{
			bFullFix = true;
			buzzer.setBuzzing(20, 3000);
		}
		auto diff = PosCalc::calcDiff(PosCalc::Position(status.gps_lat, status.gps_lon, 0.f),
			PosCalc::Position(lat, lon, alt));

		setPanDegrees(diff.bearingDeg);
		setTiltDegrees(max(diff.elevationDeg, 0.f)); // we cannot tilt downwards

		Serial.print("GPS lat: "); Serial.print(status.gps_lat, 8); Serial.print(" MAV lat: "); Serial.println(lat, 8);
		Serial.print("GPS lon: "); Serial.print(status.gps_lon, 8); Serial.print(" MAV lon: "); Serial.println(lon, 8);
		Serial.print("GPS alt: "); Serial.print(status.gps_alt); Serial.print(" MAV alt: "); Serial.println(alt);
		Serial.print("Bearing: "); Serial.print(diff.bearingDeg); Serial.print(" Elevation: "); Serial.println(diff.elevationDeg);
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

