#include "PiCar.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <chrono>

#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105

namespace PiCar
{
	using namespace std;
	using namespace cv;

	bool PiCar::Init(PICAR_MODE mode)
	{
		m_curMode = PICAR_MODE_NOT_SET;
		switch (mode)
		{
		case PICAR_MODE_DIRECT:
			if (!initCar() || !initLcd() || !initCamera())
			{
				printf("Fail to init direct mode\n");
				return false;
			}
			break;
		case PICAR_MODE_REMOTE:
			if (!initCar() || !initLcd() || !initCamera() || !initRemote())
			{
				printf("Fail to init remote mode\n");
				return false;
			}
			break;
		case PICAR_MODE_CAMERA:
			if (!initLcd() || !initCamera() || !initRemote())
			{
				printf("Fail to init camera mode\n");
				return false;
			}
			break;
		default:
			printf("Invalid mode\n");
			return false;
		}

		m_curMode = mode;
		printf("Success to init PiCar\n");
		return true;
	}
	void PiCar::Release()
	{
		switch (m_curMode)
		{
		case PICAR_MODE_DIRECT:
			m_moveMotor.Release();
			m_cameraMotor.Release();
			m_sensors.Release();
			m_cameraSensor.Release();
			break;
		case PICAR_MODE_REMOTE:
			m_moveMotor.Release();
			m_cameraMotor.Release();
			m_sensors.Release();
			m_cameraSensor.Release();
			m_subThread.join();
			m_pubThread.join();
			break;
		case PICAR_MODE_CAMERA:
			m_cameraSensor.Release();
			m_subThread.join();
			m_pubThread.join();
			break;
		default:
			break;
		}
	}
	bool PiCar::initCar()
	{
		float defaultSteerAngle = 0.0f;
		float defaultPitchAngle = 0.0f;
		float defaultYawAngle = 0.0f;
		FILE *file_r = fopen("CaliData.data", "r");
		if (file_r)
		{
			fscanf(file_r, "STEER:%f\n", &defaultSteerAngle);
			fscanf(file_r, "PITCH:%f\n", &defaultPitchAngle);
			fscanf(file_r, "YAW:%f\n", &defaultYawAngle);
			fclose(file_r);
		}
		else
		{
			FILE *file_w = fopen("CaliData.data", "w");
			if (file_w)
			{
				fprintf(file_w, "STEER:0.0\n");
				fprintf(file_w, "PITCH:0.0\n");
				fprintf(file_w, "YAW:0.0\n");
				fclose(file_w);
			}
		}

		if (!Protocol::InitCarProtocol())
		{
			printf("Fail to init CAR protocol\n");
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

		return true;
	}
	bool PiCar::initLcd()
	{
		if (!m_display.Init())
		{
			printf("Fail to init LCD protocol\n");
			return false;
		}

		Size size = m_display.GetImageSize();
		Mat temp = Mat::zeros(size, CV_8U);
		putText(temp, "test", Point(0,16), HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, 1);
		m_display.SetImage(temp);

		return true;
	}
	bool PiCar::initCamera()
	{
		int w = 1280;
		int h = 960;
		int bufSize = 120;
		int frameRate = 30;
		if (!m_cameraSensor.Init(w, h, bufSize, frameRate))
		{
			printf("Fail to init camera sensor module\n");
			return false;
		}

		return true;
	}
	bool PiCar::initRemote()
	{
		vector<string> xPubConnStrs;
		vector<string> xSubConnStrs;
		xPubConnStrs.push_back(PROXY_XPUB_STR);
		xPubConnStrs.push_back("tcp://*:45000");
		xSubConnStrs.push_back(PROXY_XSUB_STR);
		xSubConnStrs.push_back("tcp://*:45001");
		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR) ||
			!m_pubSubServer.Init(xPubConnStrs, xSubConnStrs))
		{
			printf("Fail to init proxy server\n");
			return false;
		}
		m_pubSubClient.AddSubTopic("COMMAND_PICAR");
		m_pubSubClient.ChangePubTopic("STATE_PICAR");

		return true;
	}

	bool PiCar::isConnected()
	{
		return CONNECTION_TIMEOUT.count() > chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - lastConnTime).count();
	}
	bool PiCar::updateCameraImage()
	{
		Camera::ImageInfo imgInfo;
		if (!m_cameraSensor.GetFrame(imgInfo))
			return false;

		Size printSize(640, 480);

		Mat small;
		resize(imgInfo.Image, small, printSize, 0.0, 0.0, INTER_NEAREST);

		int speed = m_moveMotor.GetRearValue();
		float steerDegree = m_moveMotor.GetSteerDegree();
		float yawDegree = m_cameraMotor.GetYawDegree();
		float pitchDegree = m_cameraMotor.GetPitchDegree();
		double distance = m_sensors.GetSonicSensorValue();
		double leftFloorVal = m_sensors.GetFloorLeftValue();
		double centorFloorVal = m_sensors.GetFloorCenterValue();
		double rightFloorVal = m_sensors.GetFloorRightValue();

		Scalar colorBlack(0, 0, 0);
		Scalar colorWhite(255, 255, 255);
		Scalar colorBlue(200, 0, 0);
		Scalar colorRed(0, 0, 200);

		{
			Point speedStrLoc(10, 10);
			Point steerStrLoc(10, 20);
			Point camYawStrLoc(10, 30);
			Point camPitchStrLoc(10, 40);

			string speedStr = format("Speed : %d", speed);
			string steerStr = format("Steer : %.01f", steerDegree);
			string camYawStr = format("Yaw : %.01f", yawDegree);
			string camPitchStr = format("Pitch : %.01f", pitchDegree);

			putText(small, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(small, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(small, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(small, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
		}
		{
			Rect speedBGRect(10, 390, 30, 80);
			Rect speedFGRect(10, 430, 30, 00);

			rectangle(small, speedBGRect, colorWhite, FILLED);
			if (speed >= 0)
			{
				speedFGRect.height = abs(speed * 0.02);
				speedFGRect.y = 430 - speedFGRect.height;
				rectangle(small, speedFGRect, colorBlue, FILLED);
			}
			else
			{
				speedFGRect.y = 430;
				speedFGRect.height = abs(speed * 0.02);
				rectangle(small, speedFGRect, colorRed, FILLED);
			}
		}
		{
			Point steerArrowFrom(80, 460);
			Point steerArrowVec(0, -50);

			double t_sin = sin(DEGREE_TO_RADIAN(steerDegree));
			double t_cos = cos(DEGREE_TO_RADIAN(steerDegree));

			Point newRotVec;
			newRotVec.x = steerArrowVec.x * t_cos - steerArrowVec.y * t_sin;
			newRotVec.y = steerArrowVec.x * t_sin + steerArrowVec.y * t_cos;
			Point steerArrowTo = steerArrowFrom + newRotVec;

			arrowedLine(small, steerArrowFrom, steerArrowTo, colorBlack, 10);
			arrowedLine(small, steerArrowFrom, steerArrowTo, colorWhite, 6);
		}
		{
			double x_cos = sin(DEGREE_TO_RADIAN(yawDegree)) * 40.0;
			double y_cos = sin(DEGREE_TO_RADIAN(pitchDegree)) * 40.0;

			Rect camBGRect(530, 390, 100, 80);
			Point camFGLoc(580 + x_cos, 435 - y_cos);

			rectangle(small, camBGRect, colorWhite, FILLED);
			circle(small, camFGLoc, 5, colorRed, FILLED);
		}
		{
			Point distanceStrLoc(320, 440);

			int baseline = 0;
			string distanceStr = format("%05.02fcm", distance * 0.1);
			Size bgSize = getTextSize(distanceStr, FONT_HERSHEY_SIMPLEX, 0.5, 5, &baseline);
			Size fgSize = getTextSize(distanceStr, FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
			Point bgStrLoc = distanceStrLoc;
			Point fgStrLoc = distanceStrLoc;
			bgStrLoc.x -= bgSize.width * 0.5;
			fgStrLoc.x -= fgSize.width * 0.5;

			putText(small, distanceStr, bgStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, colorBlack, 5);
			putText(small, distanceStr, fgStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, colorWhite, 2);
		}
		{
			Point leftLoc(300, 460);
			Point centerLoc(320, 460);
			Point rightLoc(340, 460);

			double multiLeftFloorVal = leftFloorVal / 1500.0;
			double multiCenterFloorVal = centorFloorVal / 1500.0;
			double multiRightFloorVal = rightFloorVal / 1500.0;

			if (multiLeftFloorVal > 1.0)
				multiLeftFloorVal = 1.0;
			if (multiCenterFloorVal > 1.0)
				multiCenterFloorVal = 1.0;
			if (multiRightFloorVal > 1.0)
				multiRightFloorVal = 1.0;

			Scalar leftColor(255 * multiLeftFloorVal, 255 * multiLeftFloorVal, 255);
			Scalar centerColor(255 * multiCenterFloorVal, 255 * multiLeftFloorVal, 255);
			Scalar rightColor(255 * multiRightFloorVal, 255 * multiLeftFloorVal, 255);

			circle(small, leftLoc, 5, leftColor, FILLED);
			circle(small, centerLoc, 5, centerColor, FILLED);
			circle(small, rightLoc, 5, rightColor, FILLED);
		}

		imgBufferMutex.lock();
		imgBuffer = small;
		imgBufferMutex.unlock();

		return true;
	}
	void PiCar::executeKeyInput(char ch)
	{
		int temp_i;
		float temp_f;
		switch (ch)
		{
		case 'Q':
		case 'q':
			m_isStop = true;
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
	void PiCar::subThreadFunc()
	{
		lastConnTime = chrono::steady_clock::now();
		while (!m_isStop)
		{
			zmq::multipart_t msg;
			if (m_pubSubClient.SubscribeMessage(msg) && msg.size() >= 2)
			{
				try
				{
					string topic = msg.popstr();
					string cmd = msg.popstr();

					if (cmd == "TURN_OFF")
					{
						m_isStop = true;
					}
					else if (cmd == "UPDATE_CONNECTION")
					{
						lastConnTime = chrono::steady_clock::now();
					}
				}
				catch (...)
				{
					printf("Invalid massage was detected in PiCar\n");
				}
			}

			if (m_curMode != PICAR_MODE_CAMERA && !isConnected())
				m_moveMotor.StopRearNow();

			this_thread::sleep_for(DALTA_DUATION);
		}
	}
	void PiCar::pubThreadFunc()
	{
		while (!m_isStop)
		{
			string state = !m_isStop ? "RUNNING" : "STOP";

			zmq::multipart_t pubMsg;
			pubMsg.addstr(state);
			m_pubSubClient.PublishMessage(pubMsg);

			this_thread::sleep_for(DALTA_DUATION);
		}
	}

	void PiCar::Run()
	{
		switch (m_curMode)
		{
		case PICAR_MODE_DIRECT:
			runDirectMode();
			break;
		case PICAR_MODE_REMOTE:
			runRemoteMode();
			break;
		case PICAR_MODE_CAMERA:
			runCameraMode();
			break;
		default:
			printf("Invalid mode\n");
			return;
		}
	}
	void PiCar::runDirectMode()
	{
		m_isStop = false;

		string winName = "Camera Display";
		Size winSize(960, 720);
		namedWindow(winName, WindowFlags::WINDOW_KEEPRATIO);
		resizeWindow(winName, winSize);
		while (!m_isStop)
		{
			if (updateCameraImage())
			{
				imgBufferMutex.lock();
				imshow(winName, imgBuffer);
				imgBufferMutex.unlock();
			}

			executeKeyInput(waitKey(100));
		}
		destroyWindow(winName);

		return;
	}
	void PiCar::runRemoteMode()
	{
		m_isStop = false;
		m_subThread = thread(&PiCar::subThreadFunc, this);
		m_pubThread = thread(&PiCar::pubThreadFunc, this);

		while (!m_isStop)
		{
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		return;
	}
	void PiCar::runCameraMode()
	{
		m_isStop = false;
		m_subThread = thread(&PiCar::subThreadFunc, this);
		m_pubThread = thread(&PiCar::pubThreadFunc, this);

		while (!m_isStop)
		{
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		return;
	}
}
