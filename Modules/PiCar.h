#pragma once
#include "Hardware.h"

namespace PiCar
{
	class PiCar
	{
	private:
		Hardware::MoveMotor m_moveMotor;
		Hardware::CameraMotor m_cameraMotor;
		Hardware::SonicSensor m_sonicSensor;
		Hardware::FloorSensor m_floorSensor;
		Hardware::CameraSensor m_cameraSensor;

	public:
		bool Init();
		void Release();
		
		void Run();

		void TestRearMotor();
		void TestServoMotor();
		void TestSimpleSensor();
		void TestCameraSensor();
	};
}
