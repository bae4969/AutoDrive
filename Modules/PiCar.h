#pragma once
#include "Hardware.h"
#include "INIParser.h"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <shared_mutex>
#include <thread>

namespace PiCar
{
	enum PICAR_MODE
	{
		PICAR_MODE_NOT_SET,
		PICAR_MODE_DIRECT,
		PICAR_MODE_REMOTE,
		PICAR_MODE_CAMERA,
	};

	class PiCar
	{
	private:
		PICAR_MODE m_curMode = PICAR_MODE_NOT_SET;

		Protocol::PubSubServer m_pubSubServer;
		Protocol::PubSubClient m_pubSubClient;
		Hardware::MoveMotor m_moveMotor;
		Hardware::CameraMotor m_cameraMotor;
		Hardware::Sensors m_sensors;
		Hardware::LcdDisplay m_display;
		Hardware::LidarSensor m_lidar;
		Hardware::CameraSensor m_cameraSensor;

		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(33);
		const std::chrono::milliseconds CONNECTION_TIMEOUT = std::chrono::milliseconds(3000);

		INIParser m_iniParser;
		std::atomic<bool> m_isStop;
		std::chrono::steady_clock::time_point lastConnTime;
		cv::Mat imgBuffer;
		std::shared_mutex imgBufferMutex;

		std::thread m_subThread;
		std::thread m_pubThread;

		bool isConnected();
		bool updateCameraImage();
		void executeKeyInput(char ch);
		void subThreadFunc();
		void pubThreadFunc();

		bool initBasic();
		bool initProtocol();
		bool initRobotHat();
		bool initEP0152();
		bool initLD06();
		bool initCamera();
		
		void releaseBasic();
		void releaseProtocol();
		void releaseRobotHat();
		void releaseEP0152();
		void releaseLD06();
		void releaseCamera();

		void runDirectMode();
		void runRemoteMode();
		void runCameraMode();

	public:
		bool Init(PICAR_MODE mode);
		void Release();

		void Stop();
		void Run();
	};
}
