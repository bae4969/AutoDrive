#include "Hardware.h"
#include <zmq.hpp>
#include <iostream>
#include <math.h>
#include <thread>
#include <chrono>

using namespace std;

namespace Hardware
{
#define CAMERA_STEER_MIN_DEGREE -30
#define CAMERA_STEER_MAX_DEGREE 30
#define CAMERA_PITCH_MIN_DEGREE -30
#define CAMERA_PITCH_MAX_DEGREE 50
#define CAMERA_YAW_MIN_DEGREE -45
#define CAMERA_YAW_MAX_DEGREE 45

	bool RearMotor::Init()
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
		if (!m_leftMotor.Init(Protocol::I2C_CHANNEL_REAR_LEFT, 9, 4095, 0))
		{
			printf("Fail to init rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.Init(Protocol::I2C_CHANNEL_REAR_RIGHT, 9, 4095, 0))
		{
			printf("Fail to init rear right PWM motor\n");
			return false;
		}

		return true;
	}
	bool RearMotor::SetDirection(bool isForward)
	{
		if (!m_leftDir.SetOutput(!isForward))
		{
			printf("Fail to set rear left direction");
			return false;
		}
		if (!m_rightDir.SetOutput(isForward))
		{
			printf("Fail to set rear right direction");
			return false;
		}

		m_isForward = isForward;
		return true;
	}
	bool RearMotor::SetReady(ushort maxValue)
	{
		if (0 < maxValue && maxValue <= 1000)
		{
			printf("Invalid max speed for rear PWM motors\n");
			return false;
		}
		if (maxValue == 0)
			maxValue = m_lastMaxSpeed;
		if (!SetDirection(m_isForward))
		{
			printf("Fail to ready rear direction\n");
			return false;
		}
		if (!m_leftMotor.SetMinMax(m_minSpeed, maxValue))
		{
			printf("Fail to set ready rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.SetMinMax(m_minSpeed, maxValue))
		{
			printf("Fail to set ready rear right PWM motor\n");
			return false;
		}

		m_isReady = true;
		m_lastMaxSpeed = maxValue;
		return true;
	}
	bool RearMotor::SetStop()
	{
		if (!m_leftMotor.SetMinMax(0, 1))
		{
			printf("Fail to set stop rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.SetMinMax(0, 1))
		{
			printf("Fail to set stop rear right PWM motor\n");
			return false;
		}

		m_isReady = false;
		return true;
	}
	bool RearMotor::SetThrottle(float throttle)
	{
		if (!m_isReady)
		{
			printf("Fail to set throttle rear left PWM motor\n");
		}
		if (!m_leftMotor.SetThrottle(throttle))
		{
			printf("Fail to set throttle rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.SetThrottle(throttle))
		{
			printf("Fail to set throttle rear right PWM motor\n");
			return false;
		}

		return true;
	}

	bool SteerMotor::Init(float defaultDegree)
	{
		if (!m_motor.Init(
				Protocol::I2C_CHANNEL_FRONT_STEER,
				defaultDegree,
				CAMERA_STEER_MIN_DEGREE,
				CAMERA_STEER_MAX_DEGREE))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		return true;
	}
	bool SteerMotor::SetDegreeWithTime(float degree, int millisecond)
	{
		if (!m_motor.SetDegreeWithTime(degree, millisecond))
		{
			printf("Fail to set steer servo motor\n");
			return false;
		}
		return true;
	}
	bool SteerMotor::SetDegreeWithSpeed(float degree, float absDeltaDegree){
		if (!m_motor.SetDegreeWithSpeed(degree, absDeltaDegree))
		{
			printf("Fail to set steer servo motor\n");
			return false;
		}
		return true;
	}
	float SteerMotor::GetDegree()
	{
		return m_motor.GetDegree();
	}

	bool CameraMotor::Init(float defaultPitchDegree, float defaultYawDegree)
	{
		if (!m_pitchMotor.Init(
				Protocol::I2C_CHANNEL_CAMERA_TILT,
				-defaultPitchDegree,
				-CAMERA_PITCH_MAX_DEGREE,
				-CAMERA_PITCH_MIN_DEGREE))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		if (!m_yawMotor.Init(
				Protocol::I2C_CHANNEL_CAMERA_YAW,
				-defaultYawDegree,
				-CAMERA_YAW_MAX_DEGREE,
				-CAMERA_YAW_MIN_DEGREE))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::SetPitchDegreeWithTime(float degree, int millisecond)
	{
		if (!m_pitchMotor.SetDegreeWithTime(-degree, millisecond))
		{
			printf("Fail to set camera pitch servo motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::SetPitchDegreeWithSpeed(float degree, float absDeltaDegree){
		if (!m_pitchMotor.SetDegreeWithSpeed(-degree, absDeltaDegree))
		{
			printf("Fail to set camera pitch servo motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::SetYawDegreeWithTime(float degree, int millisecond)
	{
		if (!m_yawMotor.SetDegreeWithTime(-degree, millisecond))
		{
			printf("Fail to set camera yaw servo motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::SetYawDegreeWithSpeed(float degree, float absDeltaDegree){
		if (!m_yawMotor.SetDegreeWithSpeed(-degree, absDeltaDegree))
		{
			printf("Fail to set camera yaw servo motor\n");
			return false;
		}
		return true;
	}
	float CameraMotor::GetPitchDegree()
	{
		return m_pitchMotor.GetDegree();
	}
	float CameraMotor::GetYawDegree()
	{
		return m_yawMotor.GetDegree();
	}

	bool SonicSensor::Init()
	{
		if (!m_tring.Init(Protocol::GPIO_PIN_SONIC_TRING, true))
		{
			printf("Fail to init sonic tring GPIO\n");
			return false;
		}
		if (!m_echo.Init(Protocol::GPIO_PIN_SONIC_ECHO, false))
		{
			printf("Fail to init sonic echo GPIO\n");
			return false;
		}

		return true;
	}
	bool SonicSensor::GetValue(double &value)
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
				return false;
		}
		while (m_echo.GetInput() == 1)
		{
			pulseEnd = chrono::system_clock::now();
			if (pulseEnd - timeoutStart > TIMEOUT)
				return false;
		}
		auto duration = (pulseEnd - pulseStart);
		value = duration.count() * 0.00016575; // mm
		return true;
	}

	bool FloorSensor::Init()
	{
		if (!m_left.Init(0) ||
			!m_center.Init(1) ||
			!m_right.Init(2))
		{
			printf("Fail to init floor sensor\n");
			return false;
		}

		return true;
	}
	bool FloorSensor::GetValues(ushort &left, ushort &center, ushort &right)
	{
		int t_v1 = m_left.GetValue();
		int t_v2 = m_center.GetValue();
		int t_v3 = m_right.GetValue();
		if (t_v1 < 0 || t_v2 < 0 || t_v3 < 0)
		{
			printf("Fail to get floor sensor value\n");
			return false;
		}

		left = t_v1;
		center = t_v2;
		right = t_v3;
		return true;
	}
}
