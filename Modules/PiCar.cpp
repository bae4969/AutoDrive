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

#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105

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
		if (!m_sensors.Init())
		{
			printf("Fail to init sensors\n");
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
	void PiCar::Release()
	{
		m_moveMotor.Release();
		m_sensors.Release();
		m_cameraMotor.Release();
	}

	void PiCar::Run()
	{
		bool isStop = false;
		Camera::ImageInfo img;
		Size printSize(640, 480);

		Scalar textBC(0, 0, 0);
		Scalar textFC(255, 255, 255);

		Rect speedBGRect(10, 390, 30, 80);
		Rect speedFGRect(10, 430, 30, 00);
		Scalar speedBGColor(255, 255, 255);
		Scalar speedPosColor(200, 0, 0);
		Scalar speedNegColor(0, 0, 200);
		Point steerArrowFrom(80, 460);
		Point steerArrowVec(0, -50);

		Point speedStrLoc(10, 20);
		Point steerStrLoc(10, 40);
		Point camYawStrLoc(10, 60);
		Point camPitchStrLoc(10, 80);

		Point distanceStrLoc(200, 450);

		while (!isStop)
		{
			if (m_cameraSensor.GetFrame(img))
			{
				Mat small;
				resize(img.Image, small, printSize, 0.0, 0.0, INTER_NEAREST);

				int speed = m_moveMotor.GetRearValue();
				float steerDegree = m_moveMotor.GetSteerDegree();
				float yawDegree = m_cameraMotor.GetYawDegree();
				float pitchDegree = m_cameraMotor.GetPitchDegree();
				double distance = m_sensors.GetSonicSensorValue();
				
				{
					rectangle(small, speedBGRect, speedBGColor, FILLED);
					if (speed >= 0)
					{
						speedFGRect.height = abs(speed * 0.02);
						speedFGRect.y = 430 - speedFGRect.height;
						rectangle(small, speedFGRect, speedPosColor, FILLED);
					}
					else
					{
						speedFGRect.y = 430;
						speedFGRect.height = abs(speed * 0.02);
						rectangle(small, speedFGRect, speedNegColor, FILLED);
					}
					string speedStr = format("Speed : %d", speed);
					cv::putText(small, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textBC, 5);
					cv::putText(small, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textFC, 2);
				}
				{
					double t_sin = sin(DEGREE_TO_RADIAN(steerDegree));
					double t_cos = cos(DEGREE_TO_RADIAN(steerDegree));

					Point newRotVec;
					newRotVec.x = steerArrowVec.x * t_cos - steerArrowVec.y * t_sin;
					newRotVec.y = steerArrowVec.x * t_sin + steerArrowVec.y * t_cos;
					Point steerArrowTo = steerArrowFrom + newRotVec;

					arrowedLine(small, steerArrowFrom, steerArrowTo, textBC, 10);
					arrowedLine(small, steerArrowFrom, steerArrowTo, textFC, 6);

					string steerStr = format("Steer : %.01f", steerDegree);
					cv::putText(small, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textBC, 5);
					cv::putText(small, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textFC, 2);
				}
				{
					string camYawStr = format("Yaw : %.01f", yawDegree);
					cv::putText(small, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textBC, 5);
					cv::putText(small, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textFC, 2);
				}
				{
					string camPitchStr = format("Pitch : %.01f", pitchDegree);
					cv::putText(small, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textBC, 5);
					cv::putText(small, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textFC, 2);
				}
				{
					string distanceStr = format("%05.02fcm", distance * 0.1);
					cv::putText(small, distanceStr, distanceStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textBC, 5);
					cv::putText(small, distanceStr, distanceStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, textFC, 2);
				}

				imshow("TEST", small);
			}

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
