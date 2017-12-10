#include <Arduino.h>
#include "motor.h"
#include "tools.h"

TrackerMotor::TrackerMotor(int stepPin,
	int dirPin,
	int stepsFor360Deg,
	SetupTimerCb cbSetupTimer,
	PositionCb cbPosition)
	: m_stepPin(stepPin),
	m_dirPin(dirPin),
	m_stepsFor360Deg(stepsFor360Deg),
	m_cbSetupTimer(cbSetupTimer),
	m_cbPosition(cbPosition)
{
	this->m_iMinStepsPerSec = this->degToStep(45);
	this->m_iMaxStepsPerSec = this->degToStep(240);

	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
	digitalWrite(dirPin, LOW);
}

void TrackerMotor::setContinuousSpeed(int degSec)
{
	noInterrupts();

	this->m_stepsPerSec = this->degToStep(degSec);

	this->_continue();

	interrupts();
}

void TrackerMotor::setPosition(int deg)
{
	while (deg < 0)
		deg += 360;
	while (deg >= 360)
		deg -= 360;

	noInterrupts();

	this->m_stepsPerSec = 0;
	
	this->m_targetStepPos = this->degToStep(deg);

	this->_continue();

	interrupts();
}

void TrackerMotor::onTimerElapsed()
{
	if (0 == this->m_inStep)
	{
		this->m_currentStepPos += this->m_iDir;
		while (this->m_currentStepPos < 0)
			this->m_currentStepPos += this->m_stepsFor360Deg;
		while (this->m_currentStepPos >= this->m_stepsFor360Deg)
			this->m_currentStepPos -= this->m_stepsFor360Deg;

		if (this->m_stepsPerSec != 0) 
		{
			// continuous mode: also update target position
			this->m_targetStepPos = this->m_currentStepPos;
		}

		if (nullptr != this->m_cbPosition)
			this->m_cbPosition(this->stepToDegF(this->m_currentStepPos));

		this->_continue();
	}
	else
	{
		--this->m_inStep;

		// continue with LOW period
		digitalWrite(this->m_stepPin, LOW);

		// callback after half duration
		this->m_cbSetupTimer(this->m_iStepDurationMs / 2);
	}
}

void TrackerMotor::step(int durationMs)
{
	this->m_iStepDurationMs = durationMs;

	this->m_inStep = 2;

	// begin step with HIGH period
	digitalWrite(this->m_stepPin, HIGH);

	// callback after half duration
	this->m_cbSetupTimer(durationMs / 2);
}

void TrackerMotor::_continue()
{
	if (0 == this->m_inStep)
	{
		int iTargetSpeed = this->m_stepsPerSec;
		if (0 == iTargetSpeed && this->m_targetStepPos != this->m_currentStepPos)
		{
			// calculate shorter direction
			int diff = this->m_targetStepPos - this->m_currentStepPos;
			int diff2 = -(this->m_currentStepPos + (this->m_stepsFor360Deg - this->m_targetStepPos));

			if (abs(diff2) < abs(diff))
				diff = diff2;

			iTargetSpeed = diff * 4;
			if (abs(iTargetSpeed) > this->m_iMaxStepsPerSec)
				iTargetSpeed = this->m_iMaxStepsPerSec * sgn(iTargetSpeed);
			if (abs(iTargetSpeed) < this->m_iMinStepsPerSec)
				iTargetSpeed = this->m_iMinStepsPerSec * sgn(iTargetSpeed);
		}

		if (iTargetSpeed != 0)
		{
			int iTmp = iTargetSpeed;
			int iDiff = iTargetSpeed - this->m_iFilteredTargetSpeed;
			iDiff = max(abs(iDiff) / 10, min(abs(iDiff), 5)) * sgn(iDiff);
			this->m_iFilteredTargetSpeed += iDiff;
			iTargetSpeed = this->m_iFilteredTargetSpeed;

			/*Serial.print(this->m_currentStepPos);
			Serial.print(" > ");
			Serial.print(this->m_targetStepPos);
			Serial.print(" - ");
			Serial.print(iTargetSpeed);
			Serial.print("(");
			Serial.print(iTmp);
			Serial.println(")");*/

			this->m_iDir = sgn(iTargetSpeed);
			digitalWrite(this->m_dirPin, iTargetSpeed > 0 ? HIGH : LOW);
			this->step(max(1000 / abs(iTargetSpeed), 1));
		}
	}
}


