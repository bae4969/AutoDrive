#pragma once
#include "Hardware.h"

namespace PiCar
{
	class PiCar
	{
	private:
		Hardware::MoveMotor m_moveMotor;
		Hardware::CameraMotor m_cameraMotor;
		Hardware::CameraSensor m_cameraSensor;
		Hardware::Sensors m_sensors;

	public:
		bool Init();
		void Release();
		
		void Run();

		void TestCameraSensor();
	};
}
