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
		if (!m_moveMotor.Init(defaultSteerAngle))
		{
			printf("Fail to init steer motor module\n");
			return false;
		}
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
	void PiCar::Release(){
		m_moveMotor.Release();
		m_cameraMotor.Release();
	}

	void PiCar::Run()
	{
		bool isStop = false;
		Camera::ImageInfo img;
		while (!isStop)
		{
			if (m_cameraSensor.GetFrame(img))
				imshow("TEST", img.Image);

			int temp_i;
			float temp_f;
			int keyVal = waitKey(100);
			switch (keyVal)
			{
			case 'Q':
			case 'q':
				isStop = true;
				break;

				// rear
			case 'W':
			case 'w':
				temp_i = m_moveMotor.GetRearValue() + 100;
				m_moveMotor.SetRearValue(temp_i);
				break;
			case 'S':
			case 's':
				temp_i = m_moveMotor.GetRearValue() - 100;
				m_moveMotor.SetRearValue(temp_i);
				break;
			case 'X':
			case 'x':
				m_moveMotor.SetRearValue(0);
				break;
			case 'Z':
			case 'z':
				m_moveMotor.StopRearNow();
				break;

				// steer
			case 'A':
			case 'a':
				temp_f = m_moveMotor.GetSteerDegree() - 5.0f;
				m_moveMotor.SetSteerDegree(temp_f);
				break;
			case 'D':
			case 'd':
				temp_f = m_moveMotor.GetSteerDegree() + 5.0f;
				m_moveMotor.SetSteerDegree(temp_f);
				break;
			case 'F':
			case 'f':
				m_moveMotor.SetSteerDegree(0.0f);
				break;

				// camera pitch
			case 'O':
			case 'o':
				temp_f = m_cameraMotor.GetPitchDegree() + 10.0f;
				m_cameraMotor.SetPitchDegree(temp_f);
				break;
			case 'L':
			case 'l':
				temp_f = m_cameraMotor.GetPitchDegree() - 10.0f;
				m_cameraMotor.SetPitchDegree(temp_f);
				break;
			case '.':
			case '>':
				m_cameraMotor.SetPitchDegree(0.0f);
				break;

				// camera yaw
			case 'K':
			case 'k':
				temp_f = m_cameraMotor.GetYawDegree() - 10.0f;
				m_cameraMotor.SetYawDegree(temp_f);
				break;
			case ':':
			case ';':
				temp_f = m_cameraMotor.GetYawDegree() + 10.0f;
				m_cameraMotor.SetYawDegree(temp_f);
				break;
			case 'J':
			case 'j':
				m_cameraMotor.SetYawDegree(0.0f);
				break;
			}
		}

		return;
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

		for (int i = 0; i < 3; i++)
		{
			char buf[128];
			for (int j = 0; j < 4; j++)
			{
				m_cameraMotor.SetYawDegree(-60 + j * 30);
				this_thread::sleep_for(chrono::milliseconds(500));
				m_cameraSensor.GetFrame(img);
				sprintf(buf, "./Output/AutoDrive/%d_1_%d.png", i, j);
				imwrite(buf, img.Image);
			}
			for (int j = 0; j < 4; j++)
			{
				m_cameraMotor.SetYawDegree(60 - j * 30);
				this_thread::sleep_for(chrono::milliseconds(500));
				m_cameraSensor.GetFrame(img);
				sprintf(buf, "./Output/AutoDrive/%d_2_%d.png", i, j);
				imwrite(buf, img.Image);
			}
		}
	}
}
