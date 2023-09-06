#pragma once
#include "Protocol.h"
#include "Camera.h"
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

#define PROXY_XPUB_STR "ipc://temp/SERVER_XPUB"
#define PROXY_XSUB_STR "ipc://temp/SERVER_XSUB"

namespace Hardware
{
	typedef unsigned short ushort;
	typedef unsigned char uchar;

	class MoveMotor
	{
	private:
		Protocol::PubSubClient m_pubSubClient;
		Protocol::GPIO m_leftDir;
		Protocol::GPIO m_rightDir;
		Protocol::PWMMotor m_leftMotor;
		Protocol::PWMMotor m_rightMotor;
		Protocol::ServoMotor m_steerMotor;

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

		bool updateRearValue(int value);
		bool updateSteerDegree(float degree);
		void updateThreadFunc();
		void subThreadFunc();
		void pubThreadFunc();

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
		Protocol::PubSubClient m_pubSubClient;
		Protocol::ServoMotor m_pitchMotor;
		Protocol::ServoMotor m_yawMotor;

		std::atomic<bool> m_isStop;
		std::atomic<float> m_targetPitchDegree;
		std::atomic<float> m_deltaPitchDegree;
		std::atomic<float> m_targetYawDegree;
		std::atomic<float> m_deltaYawDegree;
		std::mutex m_updateMutex;
		std::thread m_updateThread;
		std::thread m_subThread;
		std::thread m_pubThread;

		bool updatePitchDegree(float degree);
		bool updateYawDegree(float degree);
		void updateThreadFunc();
		void subThreadFunc();
		void pubThreadFunc();

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
		Protocol::PubSubClient m_pubSubClient;
		Protocol::GPIO m_tring;
		Protocol::GPIO m_echo;
		Protocol::ADC m_left;
		Protocol::ADC m_center;
		Protocol::ADC m_right;

		std::atomic<bool> m_isStop;
		std::atomic<double> m_sonicDistance; // mm
		std::atomic<int> m_floorLeftValue;
		std::atomic<int> m_floorCenterValue;
		std::atomic<int> m_floorRightValue;
		std::mutex m_floorSyncMutex;
		std::thread m_updateThread;
		std::thread m_pubThread;

		void updateSonicSensor();
		void updateFloorSensor();
		void updateThreadFunc();
		void pubThreadFunc();

	public:
		bool Init();
		void Release();
		double GetSonicSensorValue();
		int GetFloorLeftValue();
		int GetFloorCenterValue();
		int GetFloorRightValue();
	};
	class LcdDisplay : public Protocol::LCD_I2C
	{
	private:
		Protocol::PubSubClient m_pubSubClient;

		std::atomic<bool> m_isStop;
		float m_cpuTemp;
		int m_throttleState;
		std::mutex m_syncMutex;
		std::thread m_updateThread;
		std::thread m_pubThread;

		void updateThreadFunc();
		void pubThreadFunc();

	public:
		bool Init();
		void Release();
	};
	class CameraSensor : public Camera::DirectCamera
	{
	private:
		Protocol::PubSubClient m_pubSubClient;

		std::atomic<bool> m_isStop;
		std::thread m_pubThread;

		void pubRawEncodedImage();
		void pubJpgEncodedImage();
		void pubPngEncodedImage();
		void pubThreadFunc();

	public:
		bool Init(int w = 1280, int h = 960, int bufSize = 120, int frameRate = 30);
		void Release();
	};
}
