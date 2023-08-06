#pragma once
#include "Hardware.h"

namespace PiCar
{
	class PiCar
	{
	private:
		Hardware::RearMotor m_rearMotor;
		Hardware::SteerMotor m_steerMotor;
		Hardware::CameraMotor m_cameraMotor;
		Hardware::SonicSensor m_sonicSensor;
		Hardware::FloorSensor m_floorSensor;
		Hardware::CameraSensor m_cameraSensor;

	public:
		bool Init();
		void Run();

		void TestRearMotor();
		void TestServoMotor();
		void TestSimpleSensor();
		void TestCameraSensor();
	};
}
