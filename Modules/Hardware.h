#pragma once
#include "Protocol.h"
#include "Camera.h"
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

#define PROXY_XPUB_STR "ipc://SERVER_XPUB"
#define PROXY_XSUB_STR "ipc://SERVER_XSUB"

namespace Hardware
{
	typedef unsigned short ushort;
	typedef Camera::DirectCamera CameraSensor;

	class MoveMotor
	{
	private:
		Protocol::PubSubClient m_pubSub;
		Protocol::GPIO m_leftDir;
		Protocol::GPIO m_rightDir;
		Protocol::PWMMotor m_leftMotor;
		Protocol::PWMMotor m_rightMotor;
		Protocol::ServoMotor m_steerMotor;

		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(33);
		std::atomic<bool> m_isStop;
		std::atomic<int> m_targetRearValue;
		std::atomic<int> m_deltaRearValue;
		std::atomic<float> m_targetSteerDegree;
		std::atomic<float> m_deltaSteerDegree;
		std::mutex m_rearSyncMutex;
		std::mutex m_updateMutex;
		std::thread m_updateThread;
		std::thread m_subThread;
		std::thread m_pubThread;

		bool UpdateRearValue(int value);
		bool UpdateSteerDegree(float degree);
		void UpdateThreadFunc();
		void SubThreadFunc();
		void PubThreadFunc();

	public:
		bool Init(float defaultSteerDegree);
		void Release();

		bool StopRearNow();
		void SetRearValue(int value);
		void SetRearSpeed(int valuePerSecond);
		int GetRearValue();

		void SetSteerDegree(float degree);
		void SetSteerSpeed(float degreePerSecond);
		float GetSteerDegree();
	};
	class CameraMotor
	{
	private:
		Protocol::PubSubClient m_pubSub;
		Protocol::ServoMotor m_pitchMotor;
		Protocol::ServoMotor m_yawMotor;

		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(33);
		std::atomic<bool> m_isStop;
		std::atomic<float> m_targetPitchDegree;
		std::atomic<float> m_deltaPitchDegree;
		std::atomic<float> m_targetYawDegree;
		std::atomic<float> m_deltaYawDegree;
		std::mutex m_updateMutex;
		std::thread m_updateThread;
		std::thread m_subThread;
		std::thread m_pubThread;

		bool UpdatePitchDegree(float degree);
		bool UpdateYawDegree(float degree);
		void UpdateThreadFunc();
		void SubThreadFunc();
		void PubThreadFunc();

	public:
		bool Init(float defaultPitchDegree, float defaultYawDegree);
		void Release();

		void SetPitchDegree(float degree);
		void SetPitchSpeed(float degreePerSecond);
		float GetPitchDegree();

		void SetYawDegree(float degree);
		void SetYawSpeed(float degreePerSecond);
		float GetYawDegree();
	};
	class Sensors
	{
	private:
		Protocol::PubSubClient m_pubSub;
		Protocol::GPIO m_tring;
		Protocol::GPIO m_echo;
		Protocol::ADC m_left;
		Protocol::ADC m_center;
		Protocol::ADC m_right;

		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(33);
		const std::chrono::milliseconds UPDATE_PERIOD = std::chrono::milliseconds(100);
		const std::chrono::milliseconds TIMEOUT = std::chrono::milliseconds(10);
		std::atomic<bool> m_isStop;
		std::atomic<double> m_sonicDistance; // mm
		std::atomic<double> m_floorLeftValue;
		std::atomic<double> m_floorCenterValue;
		std::atomic<double> m_floorRightValue;
		std::mutex m_floorSyncMutex;
		std::thread m_updateThread;
		std::thread m_pubThread;

		void UpdateSonicSensor();
		void UpdateFloorSensor();
		void UpdateThreadFunc();
		void PubThreadFunc();

	public:
		bool Init();
		void Release();
		double GetSonicSensorValue();
		double GetFloorLeftValue();
		double GetFloorCenterValue();
		double GetFloorRightValue();
	};
}
