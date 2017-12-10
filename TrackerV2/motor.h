#ifndef MOTOR_H
#define MOTOR_H

class TrackerMotor
{
public:
	// callback must call onTimerElapsed once after iAfterMs have elapsed.
	typedef void(*SetupTimerCb)(int iAfterMs);

	// will be called for every position change
	typedef void(*PositionCb)(float fPositionDeg);

public:
	TrackerMotor(int stepPin, 
		int dirPin, 
		int stepsFor360Deg, 
		SetupTimerCb cbSetupTimer,
		PositionCb cbPosition = nullptr);

public:
	void setPosition(int deg);
	void setContinuousSpeed(int degSec);

public:
	void onTimerElapsed();

private:
	void _continue();
	void step(int durationMs);

	inline int degToStep(int deg)
	{
		return (int)((long)deg * (long)this->m_stepsFor360Deg / 360l);
	}

	inline int stepToDeg(int step)
	{
		return (int)((long)step * 360l / (long)this->m_stepsFor360Deg);
	}

	inline float stepToDegF(int step)
	{
		return (float)step * 360.f / (float)this->m_stepsFor360Deg;
	}

private:
	const int m_stepsFor360Deg;
	const int m_stepPin;
	const int m_dirPin;

	const SetupTimerCb m_cbSetupTimer;
	const PositionCb m_cbPosition;

	int m_currentStepPos = 0;
	int m_targetStepPos = 0;
	int m_stepsPerSec = 0;

	int m_iMinStepsPerSec = 20;
	int m_iMaxStepsPerSec = 200;

	int m_inStep = 0;
	int m_iStepDurationMs = 0;
	int m_iDir = 0;

	int m_iFilteredTargetSpeed = 0;
};

#endif


