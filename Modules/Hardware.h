#pragma once
#include "Protocol.h"
#include "Camera.h"
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

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
		std::atomic<bool> m_isStop;
		std::atomic<int> m_targetRearValue;
		std::atomic<int> m_deltaRearValue;
		std::atomic<float> m_targetSteerDegree;
		std::atomic<float> m_deltaSteerDegree;
		std::mutex m_rearSyncMutex;
		std::mutex m_updateMutex;
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
		std::atomic<bool> m_isStop;
		std::atomic<float> m_targetPitchDegree;
		std::atomic<float> m_deltaPitchDegree;
		std::atomic<float> m_targetYawDegree;
		std::atomic<float> m_deltaYawDegree;
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
	class Sensors
	{
	private:
		Protocol::GPIO m_tring;
		Protocol::GPIO m_echo;
		Protocol::ADC m_left;
		Protocol::ADC m_center;
		Protocol::ADC m_right;

		const std::chrono::milliseconds UPDATE_PERIOD = std::chrono::milliseconds(100);
		const std::chrono::milliseconds TIMEOUT = std::chrono::milliseconds(10);
		std::atomic<bool> m_isStop;
		std::atomic<double> m_sonicDistance; // mm
		std::atomic<double> m_floorLeftValue;
		std::atomic<double> m_floorCenterValue;
		std::atomic<double> m_floorRightValue;
		std::thread m_updateThread;

		void UpdateSonicSensor();
		void UpdateFloorSensor();
		void UpdateThreadFunc();

	public:
		bool Init();
		void Release();
		double GetSonicSensorValue();
		double GetFloorLeftValue();
		double GetFloorCenterValue();
		double GetFloorRightValue();
	};
}
