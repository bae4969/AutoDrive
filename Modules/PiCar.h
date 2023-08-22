#pragma once
#include "Hardware.h"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>

namespace PiCar
{
	class PiCar
	{
	private:
		Protocol::PubSubServer m_pubSub;
		Hardware::MoveMotor m_moveMotor;
		Hardware::CameraMotor m_cameraMotor;
		Hardware::CameraSensor m_cameraSensor;
		Hardware::Sensors m_sensors;

		std::atomic<bool> m_isStop;
		cv::Mat imgBuffer;
		std::mutex imgBufferMutex;


		bool UpdateCameraImage();
		void ExecuteKeyInput(char ch);

	public:
		bool Init();
		void Release();

		void RemoteRun();
		void DirectRun();

		void TestCameraSensor();
	};
}
