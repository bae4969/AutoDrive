#include "PiCar.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

namespace PiCar
{
	bool PiCar::Init()
	{
		float defaultSteerAngle = 0.0f;
		float defaultPitchAngle = 0.0f;
		float defaultYawAngle = 0.0f;
		FILE *file_r = fopen("CailData.data", "r");
		if (file_r)
		{
			fscanf(file_r, "STEER:%f\n", &defaultSteerAngle);
			fscanf(file_r, "PITCH:%f\n", &defaultPitchAngle);
			fscanf(file_r, "YAW:%f\n", &defaultYawAngle);
			fclose(file_r);
		}
		else
		{
			FILE *file_w = fopen("CailData.data", "w");
			if (file_w)
			{
				fprintf(file_w, "STEER:0.0\n");
				fprintf(file_w, "PITCH:0.0\n");
				fprintf(file_w, "YAW:0.0\n");
				fclose(file_w);
			}
		}

		if (!Protocol::InitProtocol())
		{
			printf("Fail to init protocol\n");
			return false;
		}
		if (!m_rearMotor.Init())
		{
			printf("Fail to init rear motor module\n");
			return false;
		};
		if (!m_steerMotor.Init(defaultSteerAngle))
		{
			printf("Fail to init steer motor module\n");
			return false;
		};
		if (!m_cameraMotor.Init(defaultPitchAngle, defaultYawAngle))
		{
			printf("Fail to init camera motor module\n");
			return false;
		}
		if (!m_sonicSensor.Init())
		{
			printf("Fail to init sonic sensor module\n");
			return false;
		}
		if (!m_floorSensor.Init())
		{
			printf("Fail to init floor sensor module\n");
			return false;
		}
		if (!m_cameraSensor.Init())
		{
			printf("Fail to init camera sensor module\n");
			return false;
		}

		printf("Success to init PiCar\n");
		return true;
	}
	void PiCar::Run()
	{
		TestCameraSensor();

		return;
	}

	void PiCar::TestRearMotor()
	{
		m_rearMotor.SetDirection(true);
		m_rearMotor.SetReady(2000);
		m_rearMotor.SetThrottle(0.3f);
		this_thread::sleep_for(chrono::seconds(1));
		m_rearMotor.SetThrottle(0.0f);
		this_thread::sleep_for(chrono::seconds(1));
		m_rearMotor.SetDirection(false);
		m_rearMotor.SetReady(2000);
		m_rearMotor.SetThrottle(0.3f);
		this_thread::sleep_for(chrono::seconds(1));
		m_rearMotor.SetStop();
	}
	void PiCar::TestServoMotor()
	{
		Hardware::SteerMotor *steerPtr = &m_steerMotor;
		Hardware::CameraMotor *cameraPtr = &m_cameraMotor;

		thread t1([steerPtr]
				  {
		steerPtr->SetDegreeWithSpeed(-1000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		steerPtr->SetDegreeWithSpeed(0, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		steerPtr->SetDegreeWithSpeed(1000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		steerPtr->SetDegreeWithSpeed(0, 30); });
		thread t2([cameraPtr]
				  {
		cameraPtr->SetPitchDegreeWithSpeed(-1000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetPitchDegreeWithSpeed(0, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetPitchDegreeWithSpeed(10000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetPitchDegreeWithSpeed(0, 30); });
		thread t3([cameraPtr]
				  {
		cameraPtr->SetYawDegreeWithSpeed(-1000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetYawDegreeWithSpeed(0, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetYawDegreeWithSpeed(1000, 30);
		this_thread::sleep_for(chrono::milliseconds(500));
		cameraPtr->SetYawDegreeWithSpeed(0, 30); });

		t1.join();
		t2.join();
		t3.join();
	}
	void PiCar::TestSimpleSensor()
	{
		double value;
		ushort t[3];
		while (true)
		{
			m_sonicSensor.GetValue(value);
			m_floorSensor.GetValues(t[0], t[1], t[2]);
			printf("Current distance : %.04fmm\n", value);
			printf("Floor value : %d, %d, %d\n", t[0], t[1], t[2]);
			this_thread::sleep_for(chrono::seconds(1));
		}
	}
	void PiCar::TestCameraSensor()
	{
		Camera::ImageInfo img;

		for(int i = 0; i < 3; i++){
			char buf[128];
			for(int j = 0; j < 4; j++){
				m_cameraMotor.SetYawDegreeWithSpeed(-60 + j * 30, 60);
				this_thread::sleep_for(chrono::milliseconds(500));
				m_cameraSensor.GetFrame(img);
				sprintf(buf, "./Output/AutoDrive/%d_1_%d.png", i, j);
				imwrite(buf, img.Image);
			}
			for(int j = 0; j < 4; j++){
				m_cameraMotor.SetYawDegreeWithSpeed(60 - j * 30, 60);
				this_thread::sleep_for(chrono::milliseconds(500));
				m_cameraSensor.GetFrame(img);
				sprintf(buf, "./Output/AutoDrive/%d_2_%d.png", i, j);
				imwrite(buf, img.Image);
			}
		}
	}
}
