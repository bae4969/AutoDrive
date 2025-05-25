#include "PiCar.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <chrono>
#include <shared_mutex>

#define DEGREE_TO_RADIAN(deg) deg * 0.0174532925199432957692369076849
#define RADIAN_TO_DEGREE(rad) rad * 57.295779513082320876798154814105

namespace PiCar
{
	using namespace std;
	using namespace cv;

	bool PiCar::Init(PICAR_MODE mode)
	{
		m_curMode = PICAR_MODE_NOT_SET;
		if (m_iniParser.Load("./AutoDrive.ini") == false)
			printf("Fail to load config file, so use default settings.\n");

		bool isGood = false;
		switch (mode)
		{
		case PICAR_MODE_DIRECT:
		case PICAR_MODE_REMOTE:
			isGood = initBasic() && initProtocol() && initRobotHat() && initEP0152() && initLD06() && initCamera();
			break;
		case PICAR_MODE_CAMERA:
			isGood = initBasic() && initProtocol() && initEP0152() && initLD06() && initCamera();
			break;
		default:
			isGood = false;
		}

		switch (mode)
		{
		case PICAR_MODE_DIRECT:
			break;
		case PICAR_MODE_REMOTE:
		case PICAR_MODE_CAMERA:
			m_subThread = thread(&PiCar::subThreadFunc, this);
			m_pubThread = thread(&PiCar::pubThreadFunc, this);
			break;
		default:
			isGood = false;
		}

		if (!isGood)
		{
			printf("Fail to init picar\n");
			return false;
		}

		m_curMode = mode;
		printf("Success to init PiCar\n");
		return true;
	}
	bool PiCar::initBasic()
	{
		if (!Basic::InitBasic())
		{
			printf("Fail to init Basic\n");
			return false;
		}

		return true;
	}
	bool PiCar::initProtocol()
	{
		if (!Protocol::InitProtocol())
		{
			printf("Fail to init protocol\n");
			return false;
		}

		string pub_ip = m_iniParser.GetValue("protocal", "publish_ip", "tcp://*:45000");
		string sub_ip = m_iniParser.GetValue("protocal", "subscribe_ip", "tcp://*:45001");

		vector<string> xPubConnStrs;
		vector<string> xSubConnStrs;
		xPubConnStrs.push_back(PROXY_XPUB_STR);
		xPubConnStrs.push_back(pub_ip);
		xSubConnStrs.push_back(PROXY_XSUB_STR);
		xSubConnStrs.push_back(sub_ip);
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
	bool PiCar::initRobotHat()
	{
		float defaultSteerAngle = m_iniParser.GetFloat("calibration", "steer_angle_offset", 0.);
		float defaultPitchAngle = m_iniParser.GetFloat("calibration", "camera_pitch_angle_offset", 0.);
		float defaultYawAngle = m_iniParser.GetFloat("calibration", "camera_yaw_angle_offset", 0.);

		if (!RobotHat::InitRobotHat())
		{
			printf("Fail to init Robot Hat\n");
			return false;
		}
		if (!m_moveMotor.Init(defaultSteerAngle))
		{
			printf("Fail to init steer motor module\n");
			return false;
		}
		// if (!m_cameraMotor.Init(defaultPitchAngle, defaultYawAngle))
		// {
		// 	printf("Fail to init camera motor module\n");
		// 	return false;
		// }
		// if (!m_sensors.Init())
		// {
		// 	printf("Fail to init sensors\n");
		// 	return false;
		// }

		return true;
	}
	bool PiCar::initEP0152()
	{
		if (!EP0152::InitEP0152())
		{

			printf("Fail to init EP0152\n");
			return false;
		}
		if (!m_display.Init())
		{
			printf("Fail to init LCD protocol\n");
			return false;
		}

		return true;
	}
	bool PiCar::initLD06()
	{
		if (!m_lidar.Init())
		{
			printf("Fail to init LD06\n");
			return false;
		}

		return true;
	}
	bool PiCar::initCamera()
	{
		int w = m_iniParser.GetInt("camera", "width", 960);
		int h = m_iniParser.GetInt("camera", "height", 720);
		int bufSize = m_iniParser.GetInt("camera", "buffer_length", 120);
		int frameRate = m_iniParser.GetInt("camera", "frame_rate", 30);
		bool is_record = m_iniParser.GetBool("camera", "is_record", false);
		if (!m_cameraSensor.Init(w, h, bufSize, frameRate, is_record))
		{
			printf("Fail to init camera sensor module\n");
			return false;
		}

		return true;
	}

	void PiCar::Release()
	{
		switch (m_curMode)
		{
		case PICAR_MODE_DIRECT:
		case PICAR_MODE_REMOTE:
			releaseCamera();
			releaseLD06();
			releaseEP0152();
			releaseRobotHat();
			releaseProtocol();
			releaseBasic();
			break;
		case PICAR_MODE_CAMERA:
			releaseCamera();
			releaseLD06();
			releaseEP0152();
			releaseProtocol();
			releaseBasic();
			break;
		default:
			break;
		}

		switch (m_curMode)
		{
		case PICAR_MODE_DIRECT:
			break;
		case PICAR_MODE_REMOTE:
		case PICAR_MODE_CAMERA:
			m_subThread.join();
			m_pubThread.join();
			break;
		default:
			break;
		}
	}
	void PiCar::releaseBasic()
	{
	}
	void PiCar::releaseProtocol()
	{
	}
	void PiCar::releaseRobotHat()
	{
		m_moveMotor.Release();
		// m_cameraMotor.Release();
		// m_sensors.Release();
	}
	void PiCar::releaseEP0152()
	{
		m_display.Release();
	}
	void PiCar::releaseLD06()
	{
		m_lidar.Release();
	}
	void PiCar::releaseCamera()
	{
		m_cameraSensor.Release();
	}

	bool PiCar::isConnected()
	{
		return CONNECTION_TIMEOUT.count() > chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - lastConnTime).count();
	}
	bool PiCar::updateCameraImage()
	{
		Camera::ImageInfo leftImageInfo, rightImageInfo;
		if (!m_cameraSensor.GetFrame(leftImageInfo, rightImageInfo))
			return false;

		Mat concat_img;
		hconcat(leftImageInfo.Image, rightImageInfo.Image, concat_img);

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

			putText(concat_img, speedStr, speedStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(concat_img, steerStr, steerStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(concat_img, camYawStr, camYawStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
			putText(concat_img, camPitchStr, camPitchStrLoc, FONT_HERSHEY_SIMPLEX, 0.3, colorWhite, 1);
		}
		{
			Rect speedBGRect(10, 390, 30, 80);
			Rect speedFGRect(10, 430, 30, 00);

			rectangle(concat_img, speedBGRect, colorWhite, FILLED);
			if (speed >= 0)
			{
				speedFGRect.height = abs(speed * 0.02);
				speedFGRect.y = 430 - speedFGRect.height;
				rectangle(concat_img, speedFGRect, colorBlue, FILLED);
			}
			else
			{
				speedFGRect.y = 430;
				speedFGRect.height = abs(speed * 0.02);
				rectangle(concat_img, speedFGRect, colorRed, FILLED);
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

			arrowedLine(concat_img, steerArrowFrom, steerArrowTo, colorBlack, 10);
			arrowedLine(concat_img, steerArrowFrom, steerArrowTo, colorWhite, 6);
		}
		{
			double x_cos = sin(DEGREE_TO_RADIAN(yawDegree)) * 40.0;
			double y_cos = sin(DEGREE_TO_RADIAN(pitchDegree)) * 40.0;

			Rect camBGRect(530, 390, 100, 80);
			Point camFGLoc(580 + x_cos, 435 - y_cos);

			rectangle(concat_img, camBGRect, colorWhite, FILLED);
			circle(concat_img, camFGLoc, 5, colorRed, FILLED);
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

			putText(concat_img, distanceStr, bgStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, colorBlack, 5);
			putText(concat_img, distanceStr, fgStrLoc, FONT_HERSHEY_SIMPLEX, 0.5, colorWhite, 2);
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

			circle(concat_img, leftLoc, 5, leftColor, FILLED);
			circle(concat_img, centerLoc, 5, centerColor, FILLED);
			circle(concat_img, rightLoc, 5, rightColor, FILLED);
		}

		{
			unique_lock lock(imgBufferMutex);
			imgBuffer = concat_img;
		}

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
		pthread_setname_np(pthread_self(), "Picar Subscriber Thread");

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
		pthread_setname_np(pthread_self(), "Picar Publisher Thread");

		while (!m_isStop)
		{
			string state = !m_isStop ? "RUNNING" : "STOP";

			zmq::multipart_t pubMsg;
			pubMsg.addstr(state);
			m_pubSubClient.PublishMessage(pubMsg);

			this_thread::sleep_for(DALTA_DUATION);
		}
	}

	void PiCar::Stop()
	{
		m_moveMotor.StopRearNow();
		m_isStop = true;
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
				shared_lock lock(imgBufferMutex);
				imshow(winName, imgBuffer);
			}

			executeKeyInput(waitKey(100));
		}
		destroyWindow(winName);

		return;
	}
	void PiCar::runRemoteMode()
	{
		m_isStop = false;

		while (!m_isStop)
		{
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		return;
	}
	void PiCar::runCameraMode()
	{
		m_isStop = false;

		while (!m_isStop)
		{
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		return;
	}
}
