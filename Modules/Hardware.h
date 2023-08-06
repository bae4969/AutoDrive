#pragma once
#include "Protocol.h"
#include "Camera.h"
#include <chrono>

namespace Hardware
{
	typedef unsigned short ushort;
	typedef Camera::DirectCamera CameraSensor;
	class RearMotor
	{
	private:
		Protocol::GPIO m_leftDir;
		Protocol::GPIO m_rightDir;
		Protocol::PWMMotor m_leftMotor;
		Protocol::PWMMotor m_rightMotor;
		bool m_isReady = false;
		bool m_isForward = true;
		const ushort m_minSpeed = 1000;
		ushort m_lastMaxSpeed = 2000;

	public:
		bool Init();
		bool SetDirection(bool isForward);
		bool SetReady(ushort maxValue = 0);
		bool SetStop();
		bool SetThrottle(float throttle);
	};
	class SteerMotor
	{
	private:
		Protocol::ServoMotor m_motor;

	public:
		bool Init(float defaultDegree);
		bool SetDegreeWithTime(float degree, int millisecond = 0);
		bool SetDegreeWithSpeed(float degree, float absDeltaDegree = 0.0);
		float GetDegree();
	};
	class CameraMotor
	{
	private:
		Protocol::ServoMotor m_pitchMotor;
		Protocol::ServoMotor m_yawMotor;

	public:
		bool Init(float defaultPitchDegree, float defaultYawDegree);
		bool SetPitchDegreeWithTime(float degree, int millisecond = 0);
		bool SetPitchDegreeWithSpeed(float degree, float absDeltaDegree = 0.0f);
		bool SetYawDegreeWithTime(float degree, int millisecond = 0);
		bool SetYawDegreeWithSpeed(float degree, float absDeltaDegree = 0.0f);
		float GetPitchDegree();
		float GetYawDegree();
	};
	class SonicSensor
	{
	private:
		const std::chrono::milliseconds TIMEOUT = std::chrono::milliseconds(10);
		Protocol::GPIO m_tring;
		Protocol::GPIO m_echo;

	public:
		bool Init();
		bool GetValue(double& value);
	};
	class FloorSensor
	{
	private:
		Protocol::ADC m_left;
		Protocol::ADC m_center;
		Protocol::ADC m_right;

	public:
		bool Init();
		bool GetValues(ushort &left, ushort &center, ushort &right);
	};
}
