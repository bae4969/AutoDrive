#include "Hardware.h"
#include <zmq.hpp>
#include <iostream>
#include <math.h>
#include <thread>
#include <chrono>

using namespace std;

#define REAR_MIN_VALUE -2000
#define REAR_MAX_VALUE 2000
#define STEER_MIN_DEGREE -30
#define STEER_MAX_DEGREE 30
#define CAMERA_PITCH_MIN_DEGREE -30
#define CAMERA_PITCH_MAX_DEGREE 50
#define CAMERA_YAW_MIN_DEGREE -60
#define CAMERA_YAW_MAX_DEGREE 60

namespace Hardware
{
	bool MoveMotor::Init(float defaultSteerAngle)
	{
		if (!m_leftDir.Init(Protocol::GPIO_PIN_REAR_LEFT_DIRECTION, true))
		{
			printf("Fail to init rear left direction GPIO\n");
			return false;
		}
		if (!m_rightDir.Init(Protocol::GPIO_PIN_REAR_RIGHT_DIRECTION, true))
		{
			printf("Fail to init rear right direction GPIO\n");
			return false;
		}
		if (!m_leftMotor.Init(Protocol::I2C_CHANNEL_REAR_LEFT))
		{
			printf("Fail to init rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.Init(Protocol::I2C_CHANNEL_REAR_RIGHT))
		{
			printf("Fail to init rear right PWM motor\n");
			return false;
		}
		if (!m_steerMotor.Init(Protocol::I2C_CHANNEL_FRONT_STEER, defaultSteerAngle))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetRearSpeed(500);
		SetRearValue(0);
		SetSteerSpeed(50.0f);
		SetSteerDegree(0.0f);
		m_updateThread = thread(&MoveMotor::UpdateThreadFunc, this);

		return true;
	}
	void MoveMotor::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		UpdateRearValue(0);
		UpdateSteerDegree(0.0f);
	}

	bool MoveMotor::UpdateRearValue(int value)
	{
		bool isForward = value >= 0;
		ushort absValue = abs(value);

		m_rearSyncMutex.lock();
		bool isGood =
			m_leftDir.SetOutput(!isForward) &&
			m_rightDir.SetOutput(isForward) &&
			m_leftMotor.SetValue(absValue) &&
			m_rightMotor.SetValue(absValue);
		m_rearSyncMutex.unlock();

		if (!isGood)
			printf("Fail to set rear value\n");

		return isGood;
	}
	bool MoveMotor::UpdateSteerDegree(float degree)
	{
		if (!m_steerMotor.SetDegree(degree))
		{
			printf("Fail to set steer servo motor\n");
			return false;
		}
		return true;
	}
	void MoveMotor::UpdateThreadFunc()
	{
		int curRearValue;
		int tarRearValue;
		int thisRearValue;
		int delDiffRearValue;
		int absDiffRearValue;

		float curSteerDegree;
		float tarSteerDegree;
		float thisSteerDegree;
		float delDiffSteerDegree;
		float absDiffSteerDegree;
		while (!m_isStop)
		{
			m_updateMutex.lock();
			curRearValue = GetRearValue();
			curSteerDegree = GetSteerDegree();
			tarRearValue = m_targetRearValue;
			tarSteerDegree = m_targetSteerDegree;
			delDiffRearValue = m_deltaRearValue;
			delDiffSteerDegree = m_deltaSteerDegree;
			m_updateMutex.unlock();

			absDiffRearValue = abs(tarRearValue - curRearValue);
			if (absDiffRearValue >= 1)
			{
				if (absDiffRearValue > delDiffRearValue)
					absDiffRearValue = delDiffRearValue;

				if (tarRearValue > curRearValue)
					thisRearValue = curRearValue + absDiffRearValue;
				else
					thisRearValue = curRearValue - absDiffRearValue;

				UpdateRearValue(thisRearValue);
			}

			absDiffSteerDegree = abs(tarSteerDegree - curSteerDegree);
			if (absDiffSteerDegree >= FLT_EPSILON)
			{
				if (absDiffSteerDegree > delDiffSteerDegree)
					absDiffSteerDegree = delDiffSteerDegree;

				if (tarSteerDegree > curSteerDegree)
					thisSteerDegree = curSteerDegree + absDiffSteerDegree;
				else
					thisSteerDegree = curSteerDegree - absDiffSteerDegree;

				UpdateSteerDegree(thisSteerDegree);
			}

			this_thread::sleep_for(DALTA_DUATION);
		}
	}

	bool MoveMotor::StopRearNow()
	{
		m_updateMutex.lock();
		m_targetRearValue = 0;
		m_updateMutex.unlock();
		return UpdateRearValue(m_targetRearValue);
	}
	void MoveMotor::SetRearSpeed(int valuePerSecond)
	{
		m_updateMutex.lock();
		m_deltaRearValue = abs(round(valuePerSecond * 0.01));
		m_updateMutex.unlock();
	}
	void MoveMotor::SetRearValue(int value)
	{
		if (value > REAR_MAX_VALUE)
			value = REAR_MAX_VALUE;
		if (value < REAR_MIN_VALUE)
			value = REAR_MIN_VALUE;

		m_updateMutex.lock();
		m_targetRearValue = value;
		m_updateMutex.unlock();
	}
	int MoveMotor::GetRearValue()
	{
		int value = m_leftMotor.GetValue();
		if (m_leftDir.GetOutput() > 0)
			value = -value;
		return value;
	}

	void MoveMotor::SetSteerSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaSteerDegree = abs(degreePerSecond * 0.01f);
		m_updateMutex.unlock();
	}
	void MoveMotor::SetSteerDegree(float degree)
	{
		if (degree > STEER_MAX_DEGREE)
			degree = STEER_MAX_DEGREE;
		if (degree < STEER_MIN_DEGREE)
			degree = STEER_MIN_DEGREE;

		m_updateMutex.lock();
		m_targetSteerDegree = degree;
		m_updateMutex.unlock();
	}
	float MoveMotor::GetSteerDegree()
	{
		return m_steerMotor.GetDegree();
	}

	bool CameraMotor::Init(float defaultPitchDegree, float defaultYawDegree)
	{
		if (!m_pitchMotor.Init(Protocol::I2C_CHANNEL_CAMERA_TILT, -defaultPitchDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		if (!m_yawMotor.Init(Protocol::I2C_CHANNEL_CAMERA_YAW, -defaultYawDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetPitchSpeed(50.0f);
		SetPitchDegree(0.0f);
		SetYawSpeed(50.0f);
		SetYawDegree(0.0f);
		m_updateThread = thread(&CameraMotor::UpdateThreadFunc, this);

		return true;
	}
	void CameraMotor::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		UpdatePitchDegree(0.0f);
		UpdateYawDegree(0.0f);
	}

	bool CameraMotor::UpdatePitchDegree(float degree)
	{
		if (!m_pitchMotor.SetDegree(-degree))
		{
			printf("Fail to set camera pitch servo motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::UpdateYawDegree(float degree)
	{
		if (!m_yawMotor.SetDegree(-degree))
		{
			printf("Fail to set camera yaw servo motor\n");
			return false;
		}
		return true;
	}
	void CameraMotor::UpdateThreadFunc()
	{
		float curPitchDegree;
		float tarPitchDegree;
		float thisPitchDegree;
		float delDiffPitchDegree;
		float absDiffPitchDegree;

		float curYawDegree;
		float tarYawDegree;
		float thisYawDegree;
		float delDiffYawDegree;
		float absDiffYawDegree;
		while (!m_isStop)
		{
			m_updateMutex.lock();
			curPitchDegree = GetPitchDegree();
			curYawDegree = GetYawDegree();
			tarPitchDegree = m_targetPitchDegree;
			tarYawDegree = m_targetYawDegree;
			delDiffPitchDegree = m_deltaPitchDegree;
			delDiffYawDegree = m_deltaYawDegree;
			m_updateMutex.unlock();

			absDiffPitchDegree = abs(tarPitchDegree - curPitchDegree);
			if (absDiffPitchDegree >= FLT_EPSILON)
			{
				if (absDiffPitchDegree > delDiffPitchDegree)
					absDiffPitchDegree = delDiffPitchDegree;

				if (tarPitchDegree > curPitchDegree)
					thisPitchDegree = curPitchDegree + absDiffPitchDegree;
				else
					thisPitchDegree = curPitchDegree - absDiffPitchDegree;

				UpdatePitchDegree(thisPitchDegree);
			}

			absDiffYawDegree = abs(tarYawDegree - curYawDegree);
			if (absDiffYawDegree >= FLT_EPSILON)
			{
				if (absDiffYawDegree > delDiffYawDegree)
					absDiffYawDegree = delDiffYawDegree;

				if (tarYawDegree > curYawDegree)
					thisYawDegree = curYawDegree + absDiffYawDegree;
				else
					thisYawDegree = curYawDegree - absDiffYawDegree;

				UpdateYawDegree(thisYawDegree);
			}

			this_thread::sleep_for(DALTA_DUATION);
		}
	}

	void CameraMotor::SetPitchSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaPitchDegree = abs(degreePerSecond * 0.01f);
		m_updateMutex.unlock();
	}
	void CameraMotor::SetPitchDegree(float degree)
	{
		if (degree > CAMERA_PITCH_MAX_DEGREE)
			degree = CAMERA_PITCH_MAX_DEGREE;
		if (degree < CAMERA_PITCH_MIN_DEGREE)
			degree = CAMERA_PITCH_MIN_DEGREE;

		m_updateMutex.lock();
		m_targetPitchDegree = degree;
		m_updateMutex.unlock();
	}
	float CameraMotor::GetPitchDegree()
	{
		return -m_pitchMotor.GetDegree();
	}

	void CameraMotor::SetYawSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaYawDegree = abs(degreePerSecond * 0.01f);
		m_updateMutex.unlock();
	}
	void CameraMotor::SetYawDegree(float degree)
	{
		if (degree > CAMERA_YAW_MAX_DEGREE)
			degree = CAMERA_YAW_MAX_DEGREE;
		if (degree < CAMERA_YAW_MIN_DEGREE)
			degree = CAMERA_YAW_MIN_DEGREE;

		m_updateMutex.lock();
		m_targetYawDegree = degree;
		m_updateMutex.unlock();
	}
	float CameraMotor::GetYawDegree()
	{
		return -m_yawMotor.GetDegree();
	}

	bool Sensors::Init()
	{
		if (!m_tring.Init(Protocol::GPIO_PIN_SONIC_TRING, true) ||
			!m_echo.Init(Protocol::GPIO_PIN_SONIC_ECHO, false))
		{
			printf("Fail to init sonic sensor GPIO\n");
			return false;
		}
		if (!m_left.Init(0) ||
			!m_center.Init(1) ||
			!m_right.Init(2))
		{
			printf("Fail to init floor sensor\n");
			return false;
		}

		m_isStop = false;
		m_sonicDistance = -1.0;
		m_floorLeftValue = 0;
		m_floorCenterValue = 0;
		m_floorRightValue = 0;
		m_updateThread = thread(&Sensors::UpdateThreadFunc, this);

		return true;
	}
	void Sensors::Release()
	{
		m_isStop = true;
		m_updateThread.join();
	}

	void Sensors::UpdateSonicSensor()
	{
		m_tring.SetOutput(false);
		this_thread::sleep_for(chrono::milliseconds(10));
		m_tring.SetOutput(true);
		this_thread::sleep_for(chrono::nanoseconds(10));
		m_tring.SetOutput(false);
		auto timeoutStart = chrono::system_clock::now();
		auto pulseStart = timeoutStart;
		auto pulseEnd = timeoutStart;
		while (m_echo.GetInput() == 0)
		{
			pulseStart = chrono::system_clock::now();
			if (pulseStart - timeoutStart > TIMEOUT)
				return;
		}
		while (m_echo.GetInput() == 1)
		{
			pulseEnd = chrono::system_clock::now();
			if (pulseEnd - timeoutStart > TIMEOUT)
				return;
		}
		auto duration = (pulseEnd - pulseStart);
		m_sonicDistance = duration.count() * 0.00016575; // mm
	}
	void Sensors::UpdateFloorSensor()
	{
		int t_v1 = m_left.GetValue();
		int t_v2 = m_center.GetValue();
		int t_v3 = m_right.GetValue();
		if (t_v1 < 0 || t_v2 < 0 || t_v3 < 0)
			return;

		m_floorLeftValue = t_v1;
		m_floorCenterValue = t_v2;
		m_floorRightValue = t_v3;
	}
	void Sensors::UpdateThreadFunc()
	{
		while (!m_isStop)
		{
			UpdateSonicSensor();
			// UpdateFloorSensor();
			this_thread::sleep_for(UPDATE_PERIOD);
		}
	}

	double Sensors::GetSonicSensorValue()
	{
		return m_sonicDistance;
	}
	ushort Sensors::GetFloorLeftValue()
	{
		return m_floorLeftValue;
	}
	ushort Sensors::GetFloorCenterValue()
	{
		return m_floorCenterValue;
	}
	ushort Sensors::GetFloorRightValue()
	{
		return m_floorRightValue;
	}
}
