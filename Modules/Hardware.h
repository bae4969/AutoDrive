#pragma once
#include "Protocol.h"
#include "Camera.h"
#include <chrono>
#include <mutex>
#include <thread>

namespace Hardware
{
	typedef unsigned short ushort;
	typedef Camera::DirectCamera CameraSensor;
	class MoveMotor
	{
	private:
		Protocol::GPIO m_leftDir;
		Protocol::GPIO m_rightDir;
		Protocol::PWMMotor m_leftMotor;
		Protocol::PWMMotor m_rightMotor;
		Protocol::ServoMotor m_steerMotor;
		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(10);
		bool m_isStop;
		int m_targetRearValue;
		int m_deltaRearValue;
		float m_targetSteerDegree;
		float m_deltaSteerDegree;
		std::mutex m_updateMutex;
		std::mutex m_rearMutex;
		std::thread m_updateThread;

		bool UpdateRearValue(int value);
		bool UpdateSteerDegree(float degree);
		void UpdateThreadFunc();

	public:
		bool Init(float defaultSteerDegree);
		void Release();

		bool StopRearNow();
		void SetRearSpeed(int valuePerSecond);
		void SetRearValue(int value);
		int GetRearValue();

		void SetSteerSpeed(float degreePerSecond);
		void SetSteerDegree(float degree);
		float GetSteerDegree();
	};
	class CameraMotor
	{
	private:
		Protocol::ServoMotor m_pitchMotor;
		Protocol::ServoMotor m_yawMotor;
		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(10);
		bool m_isStop;
		float m_targetPitchDegree;
		float m_deltaPitchDegree;
		float m_targetYawDegree;
		float m_deltaYawDegree;
		std::mutex m_updateMutex;
		std::thread m_updateThread;

		bool UpdatePitchDegree(float degree);
		bool UpdateYawDegree(float degree);
		void UpdateThreadFunc();

	public:
		bool Init(float defaultPitchDegree, float defaultYawDegree);
		void Release();

		void SetPitchSpeed(float degreePerSecond);
		void SetPitchDegree(float degree);
		float GetPitchDegree();

		void SetYawSpeed(float degreePerSecond);
		void SetYawDegree(float degree);
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
