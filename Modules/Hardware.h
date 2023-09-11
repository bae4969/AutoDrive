#pragma once
#include "Basic.h"
#include "Protocol.h"
#include "RobotHat.h"
#include "EP0152.h"
#include "Camera.h"

namespace Hardware
{
	typedef unsigned short ushort;
	typedef unsigned char uchar;

	class MoveMotor
	{
	private:
		Protocol::PubSubClient m_pubSubClient;
		Basic::GPIO m_leftDir;
		Basic::GPIO m_rightDir;
		RobotHat::PWMMotor m_leftMotor;
		RobotHat::PWMMotor m_rightMotor;
		RobotHat::ServoMotor m_steerMotor;

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
		RobotHat::ServoMotor m_pitchMotor;
		RobotHat::ServoMotor m_yawMotor;

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
		Basic::GPIO m_led;
		Basic::GPIO m_switch;
		Basic::GPIO m_tring;
		Basic::GPIO m_echo;
		RobotHat::ADC m_left;
		RobotHat::ADC m_center;
		RobotHat::ADC m_right;

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
	class LcdDisplay : public EP0152::LCD_I2C
	{
	private:
		Protocol::PubSubClient m_pubSubClient;
		Basic::GPIO m_ledFrontLeft;
		Basic::GPIO m_ledFrontRight;
		Basic::GPIO m_ledBackLeft;
		Basic::GPIO m_ledBackRight;

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
