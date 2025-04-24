#include "Hardware.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <shared_mutex>

#define REAR_MIN_VALUE -2000
#define REAR_MAX_VALUE 2000
#define STEER_MIN_DEGREE -30
#define STEER_MAX_DEGREE 30
#define CAMERA_PITCH_MIN_DEGREE -30
#define CAMERA_PITCH_MAX_DEGREE 50
#define CAMERA_YAW_MIN_DEGREE -70
#define CAMERA_YAW_MAX_DEGREE 70

namespace Hardware
{
	using namespace std;
	using namespace cv;

	static const chrono::milliseconds MOTION_DALTA_DUATION = chrono::milliseconds(16);
	static const chrono::milliseconds LCD_DALTA_DUATION = chrono::milliseconds(1000);
	static const chrono::milliseconds LIDAR_DALTA_DUATION = chrono::milliseconds(33);

	bool MoveMotor::Init(float defaultSteerAngle)
	{
		if (!m_leftDir.Init(Basic::GPIO_PIN_REAR_LEFT_DIRECTION, true))
		{
			printf("Fail to init rear left direction GPIO\n");
			return false;
		}
		if (!m_rightDir.Init(Basic::GPIO_PIN_REAR_RIGHT_DIRECTION, true))
		{
			printf("Fail to init rear right direction GPIO\n");
			return false;
		}
		if (!m_leftMotor.Init(RobotHat::CAR_I2C_CHANNEL_REAR_LEFT))
		{
			printf("Fail to init rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.Init(RobotHat::CAR_I2C_CHANNEL_REAR_RIGHT))
		{
			printf("Fail to init rear right PWM motor\n");
			return false;
		}
		if (!m_steerMotor.Init(RobotHat::CAR_I2C_CHANNEL_FRONT_STEER, defaultSteerAngle))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetRearSpeed(1000);
		SetRearValue(0);
		SetSteerSpeed(30.0f);
		SetSteerDegree(0.0f);
		m_updateThread = thread(&MoveMotor::updateThreadFunc, this);

		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for move motor\n");
			return false;
		}
		m_pubSubClient.AddSubTopic("COMMAND_MOVE_MOTOR");
		m_pubSubClient.ChangePubTopic("STATE_MOVE_MOTOR");
		m_subThread = thread(&MoveMotor::subThreadFunc, this);
		m_pubThread = thread(&MoveMotor::pubThreadFunc, this);

		return true;
	}
	void MoveMotor::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		m_subThread.join();
		m_pubThread.join();
		updateRearValue(0);
		updateSteerDegree(0.0f);
	}

	bool MoveMotor::updateRearValue(int value)
	{
		bool isForward = value >= 0;
		ushort absValue = abs(value);
		bool isGood;

		{
			unique_lock lock(m_rearSyncMutex);
			isGood =
				m_leftDir.SetOutput(!isForward) &&
				m_rightDir.SetOutput(isForward) &&
				m_leftMotor.SetValue(absValue) &&
				m_rightMotor.SetValue(absValue);
		}

		if (!isGood)
			printf("Fail to set rear value\n");

		return isGood;
	}
	bool MoveMotor::updateSteerDegree(float degree)
	{
		if (!m_steerMotor.SetDegree(degree))
		{
			printf("Fail to set steer servo motor\n");
			return false;
		}
		return true;
	}
	void MoveMotor::updateThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Move Motor Updater Thread");

		chrono::steady_clock::time_point start;

		int curRearValue;
		int tarRearValue;
		int thisRearValue;
		int delDiffRearValue;
		int absDiffRearValue;

		float curSteerDegree;
		float tarSteerDegree;
		float thisSteerDegree;
		float delDiffSteerDegree;
		float absDiffSteerDegree;
		while (!m_isStop)
		{
			{
				shared_lock lock(m_updateMutex);
				start = chrono::steady_clock::now();
				curRearValue = GetRearValue();
				curSteerDegree = GetSteerDegree();
				tarRearValue = m_targetRearValue;
				tarSteerDegree = m_targetSteerDegree;
				delDiffRearValue = m_deltaRearValue;
				delDiffSteerDegree = m_deltaSteerDegree;
			}

			absDiffRearValue = abs(tarRearValue - curRearValue);
			if (absDiffRearValue >= 1)
			{
				if (absDiffRearValue > delDiffRearValue)
					absDiffRearValue = delDiffRearValue;

				if (tarRearValue > curRearValue)
					thisRearValue = curRearValue + absDiffRearValue;
				else
					thisRearValue = curRearValue - absDiffRearValue;

				updateRearValue(thisRearValue);
			}

			absDiffSteerDegree = abs(tarSteerDegree - curSteerDegree);
			if (absDiffSteerDegree >= FLT_EPSILON)
			{
				if (absDiffSteerDegree > delDiffSteerDegree)
					absDiffSteerDegree = delDiffSteerDegree;

				if (tarSteerDegree > curSteerDegree)
					thisSteerDegree = curSteerDegree + absDiffSteerDegree;
				else
					thisSteerDegree = curSteerDegree - absDiffSteerDegree;

				updateSteerDegree(thisSteerDegree);
			}

			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void MoveMotor::subThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Move Motor Subscriber Thread");

		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			try
			{
				zmq::multipart_t msg;
				if (!m_pubSubClient.SubscribeMessage(msg) || msg.size() < 4)
					continue;

				string topic = msg.popstr();
				string cmd = msg.popstr();
				string type = msg.popstr();

				if (cmd == "REAR_MOTOR")
				{
					int val = msg.poptyp<int>();
					if (type == "VALUE")
						SetRearValue(val);
					else if (type == "SPEED")
						SetRearSpeed(val);
					else if (type == "STOP")
						StopRearNow();
				}
				else if (cmd == "STEER_MOTOR")
				{
					float val = msg.poptyp<float>();
					if (type == "VALUE")
						SetSteerDegree(val);
					else if (type == "SPEED")
						SetSteerSpeed(val);
				}
			}
			catch (...)
			{
				printf("Invalid massage was detected in MoveMotor\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void MoveMotor::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Move Motor Publisher Thread");

		chrono::steady_clock::time_point start;
		int curRearValue;
		int tarRearValue;
		int delDiffRearValue;
		float curSteerDegree;
		float tarSteerDegree;
		float delDiffSteerDegree;

		while (!m_isStop)
		{
			{
				shared_lock lock(m_updateMutex);
				start = chrono::steady_clock::now();
				curRearValue = GetRearValue();
				tarRearValue = m_targetRearValue;
				delDiffRearValue = m_deltaRearValue;
				curSteerDegree = GetSteerDegree();
				tarSteerDegree = m_targetSteerDegree;
				delDiffSteerDegree = m_deltaSteerDegree;
			}

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp(curRearValue);
				pubMsg.addtyp(tarRearValue);
				pubMsg.addtyp(delDiffRearValue);
				pubMsg.addtyp(curSteerDegree);
				pubMsg.addtyp(tarSteerDegree);
				pubMsg.addtyp(delDiffSteerDegree);
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in MoveMotor\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}

	bool MoveMotor::StopRearNow()
	{
		unique_lock lock(m_updateMutex);
		m_targetRearValue = 0;
		return updateRearValue(m_targetRearValue);
	}
	void MoveMotor::SetRearValue(int value)
	{
		if (value > REAR_MAX_VALUE)
			value = REAR_MAX_VALUE;
		if (value < REAR_MIN_VALUE)
			value = REAR_MIN_VALUE;

		unique_lock lock(m_updateMutex);
		m_targetRearValue = value;
	}
	void MoveMotor::SetRearSpeed(int valuePerSecond)
	{
		unique_lock lock(m_updateMutex);
		m_deltaRearValue = abs(round(valuePerSecond * MOTION_DALTA_DUATION.count() / 1000.0));
	}
	int MoveMotor::GetRearValue()
	{
		int value = m_leftMotor.GetValue();
		if (m_leftDir.GetOutput() > 0)
			value = -value;
		return value;
	}

	void MoveMotor::SetSteerDegree(float degree)
	{
		if (degree > STEER_MAX_DEGREE)
			degree = STEER_MAX_DEGREE;
		if (degree < STEER_MIN_DEGREE)
			degree = STEER_MIN_DEGREE;

		unique_lock lock(m_updateMutex);
		m_targetSteerDegree = degree;
	}
	void MoveMotor::SetSteerSpeed(float degreePerSecond)
	{
		unique_lock lock(m_updateMutex);
		m_deltaSteerDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
	}
	float MoveMotor::GetSteerDegree()
	{
		return m_steerMotor.GetDegree();
	}

	bool CameraMotor::Init(float defaultPitchDegree, float defaultYawDegree)
	{
		if (!m_pitchMotor.Init(RobotHat::CAR_I2C_CHANNEL_CAMERA_TILT, -defaultPitchDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		if (!m_yawMotor.Init(RobotHat::CAR_I2C_CHANNEL_CAMERA_YAW, -defaultYawDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetPitchSpeed(90.0f);
		SetPitchDegree(0.0f);
		SetYawSpeed(90.0f);
		SetYawDegree(0.0f);
		m_updateThread = thread(&CameraMotor::updateThreadFunc, this);

		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for camera motor\n");
			return false;
		}
		m_pubSubClient.AddSubTopic("COMMAND_CAMERA_MOTOR");
		m_pubSubClient.ChangePubTopic("STATE_CAMERA_MOTOR");
		m_subThread = thread(&CameraMotor::subThreadFunc, this);
		m_pubThread = thread(&CameraMotor::pubThreadFunc, this);

		return true;
	}
	void CameraMotor::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		m_subThread.join();
		m_pubThread.join();
		updatePitchDegree(0.0f);
		updateYawDegree(0.0f);
	}

	bool CameraMotor::updatePitchDegree(float degree)
	{
		if (!m_pitchMotor.SetDegree(-degree))
		{
			printf("Fail to set camera pitch servo motor\n");
			return false;
		}
		return true;
	}
	bool CameraMotor::updateYawDegree(float degree)
	{
		if (!m_yawMotor.SetDegree(-degree))
		{
			printf("Fail to set camera yaw servo motor\n");
			return false;
		}
		return true;
	}
	void CameraMotor::updateThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Camera Motor Updater Thread");

		chrono::steady_clock::time_point start;

		float curPitchDegree;
		float tarPitchDegree;
		float thisPitchDegree;
		float delDiffPitchDegree;
		float absDiffPitchDegree;

		float curYawDegree;
		float tarYawDegree;
		float thisYawDegree;
		float delDiffYawDegree;
		float absDiffYawDegree;

		while (!m_isStop)
		{
			{
				shared_lock lock(m_updateMutex);
				start = chrono::steady_clock::now();
				curPitchDegree = GetPitchDegree();
				curYawDegree = GetYawDegree();
				tarPitchDegree = m_targetPitchDegree;
				tarYawDegree = m_targetYawDegree;
				delDiffPitchDegree = m_deltaPitchDegree;
				delDiffYawDegree = m_deltaYawDegree;
			}

			absDiffPitchDegree = abs(tarPitchDegree - curPitchDegree);
			if (absDiffPitchDegree >= FLT_EPSILON)
			{
				if (absDiffPitchDegree > delDiffPitchDegree)
					absDiffPitchDegree = delDiffPitchDegree;

				if (tarPitchDegree > curPitchDegree)
					thisPitchDegree = curPitchDegree + absDiffPitchDegree;
				else
					thisPitchDegree = curPitchDegree - absDiffPitchDegree;

				updatePitchDegree(thisPitchDegree);
			}

			absDiffYawDegree = abs(tarYawDegree - curYawDegree);
			if (absDiffYawDegree >= FLT_EPSILON)
			{

				if (absDiffYawDegree > delDiffYawDegree)
					absDiffYawDegree = delDiffYawDegree;

				if (tarYawDegree > curYawDegree)
					thisYawDegree = curYawDegree + absDiffYawDegree;
				else
					thisYawDegree = curYawDegree - absDiffYawDegree;

				updateYawDegree(thisYawDegree);
			}

			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void CameraMotor::subThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Camera Motor Subscriber Thread");

		chrono::steady_clock::time_point start;
		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			try
			{
				zmq::multipart_t msg;
				if (!m_pubSubClient.SubscribeMessage(msg) || msg.size() < 4)
					continue;

				string topic = msg.popstr();
				string cmd = msg.popstr();
				string type = msg.popstr();
				float val = msg.poptyp<float>();

				if (cmd == "PITCH_MOTOR")
				{
					if (type == "VALUE")
						SetPitchDegree(val);
					else if (type == "SPEED")
						SetPitchSpeed(val);
				}
				else if (cmd == "YAW_MOTOR")
				{
					if (type == "VALUE")
						SetYawDegree(val);
					else if (type == "SPEED")
						SetYawSpeed(val);
				}
			}
			catch (...)
			{
				printf("Invalid massage was detected in CameraMotor\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void CameraMotor::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Camera Motor Publisher Thread");

		chrono::steady_clock::time_point start;
		float curPitchDegree;
		float tarPitchDegree;
		float delDiffPitchDegree;
		float curYawDegree;
		float tarYawDegree;
		float delDiffYawDegree;

		while (!m_isStop)
		{
			{
				shared_lock lock(m_updateMutex);
				start = chrono::steady_clock::now();
				curPitchDegree = GetPitchDegree();
				tarPitchDegree = m_targetPitchDegree;
				delDiffPitchDegree = m_deltaPitchDegree;
				curYawDegree = GetYawDegree();
				tarYawDegree = m_targetYawDegree;
				delDiffYawDegree = m_deltaYawDegree;
			}

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp(curPitchDegree);
				pubMsg.addtyp(tarPitchDegree);
				pubMsg.addtyp(delDiffPitchDegree);
				pubMsg.addtyp(curYawDegree);
				pubMsg.addtyp(tarYawDegree);
				pubMsg.addtyp(delDiffYawDegree);
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in CameraMotor\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}

	void CameraMotor::SetPitchDegree(float degree)
	{
		if (degree > CAMERA_PITCH_MAX_DEGREE)
			degree = CAMERA_PITCH_MAX_DEGREE;
		if (degree < CAMERA_PITCH_MIN_DEGREE)
			degree = CAMERA_PITCH_MIN_DEGREE;

		unique_lock lock(m_updateMutex);
		m_targetPitchDegree = degree;
	}
	void CameraMotor::SetPitchSpeed(float degreePerSecond)
	{
		unique_lock lock(m_updateMutex);
		m_deltaPitchDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
	}
	float CameraMotor::GetPitchDegree()
	{
		return -m_pitchMotor.GetDegree();
	}

	void CameraMotor::SetYawDegree(float degree)
	{
		if (degree > CAMERA_YAW_MAX_DEGREE)
			degree = CAMERA_YAW_MAX_DEGREE;
		if (degree < CAMERA_YAW_MIN_DEGREE)
			degree = CAMERA_YAW_MIN_DEGREE;

		unique_lock lock(m_updateMutex);
		m_targetYawDegree = degree;
	}
	void CameraMotor::SetYawSpeed(float degreePerSecond)
	{
		unique_lock lock(m_updateMutex);
		m_deltaYawDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
	}
	float CameraMotor::GetYawDegree()
	{
		return -m_yawMotor.GetDegree();
	}

	bool Sensors::Init()
	{
		if (!m_led.Init(Basic::GPIO_PIN_CAR_LED, true) ||
			!m_switch.Init(Basic::GPIO_PIN_SWITCH, false) ||
			!m_tring.Init(Basic::GPIO_PIN_SONIC_TRING, true) ||
			!m_echo.Init(Basic::GPIO_PIN_SONIC_ECHO, false))
		{
			printf("Fail to init sonic sensor GPIO\n");
			return false;
		}
		if (!m_left.Init(0) ||
			!m_center.Init(1) ||
			!m_right.Init(2))
		{
			printf("Fail to init floor sensor\n");
			return false;
		}

		m_isStop = false;
		m_led.SetOutput(true);
		m_sonicDistance = -1.0;
		m_floorLeftValue = 0;
		m_floorCenterValue = 0;
		m_floorRightValue = 0;
		m_updateThread = thread(&Sensors::updateThreadFunc, this);

		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for sensor\n");
			return false;
		}
		m_pubSubClient.ChangePubTopic("STATE_SENSOR");
		m_pubThread = thread(&Sensors::pubThreadFunc, this);

		return true;
	}
	void Sensors::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		m_pubThread.join();
	}

	void Sensors::updateSonicSensor()
	{
		const std::chrono::milliseconds TIMEOUT = std::chrono::milliseconds(10);

		m_tring.SetOutput(false);
		this_thread::sleep_for(chrono::microseconds(2));
		m_tring.SetOutput(true);
		this_thread::sleep_for(chrono::microseconds(10));
		m_tring.SetOutput(false);
		auto timeoutStart = chrono::steady_clock::now();
		auto pulseStart = timeoutStart;
		auto pulseEnd = timeoutStart;
		while (m_echo.GetInput() == 0)
		{
			pulseStart = chrono::steady_clock::now();
			if (pulseStart - timeoutStart > TIMEOUT)
				return;
		}
		while (m_echo.GetInput() == 1)
		{
			pulseEnd = chrono::steady_clock::now();
			if (pulseEnd - timeoutStart > TIMEOUT)
				return;
		}
		auto duration = (pulseEnd - pulseStart);
		m_sonicDistance = duration.count() * 0.00016575; // mm
		m_led.SetOutput(m_sonicDistance < 50.0);
	}
	void Sensors::updateFloorSensor()
	{
		int t_v1 = m_left.GetValue();
		int t_v2 = m_center.GetValue();
		int t_v3 = m_right.GetValue();
		if (t_v1 < 0 || t_v2 < 0 || t_v3 < 0)
			return;

		{
			unique_lock lock(m_floorSyncMutex);
			m_floorLeftValue = t_v1;
			m_floorCenterValue = t_v2;
			m_floorRightValue = t_v3;
		}
	}
	void Sensors::updateThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Sensors Updater Thread");

		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();

			updateSonicSensor();
			updateFloorSensor();

			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void Sensors::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Sensors Publisher Thread");

		chrono::steady_clock::time_point start;
		double sonicValue;
		int floorLeftValue;
		int floorCenterValue;
		int floorRightValue;

		while (!m_isStop)
		{
			{
				shared_lock lock(m_floorSyncMutex);
				start = chrono::steady_clock::now();
				sonicValue = m_sonicDistance;
				floorLeftValue = m_floorLeftValue;
				floorCenterValue = m_floorCenterValue;
				floorRightValue = m_floorRightValue;
			}

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp(sonicValue);
				pubMsg.addtyp(floorLeftValue);
				pubMsg.addtyp(floorCenterValue);
				pubMsg.addtyp(floorRightValue);
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in Sensor\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}

	double Sensors::GetSonicSensorValue()
	{
		return m_sonicDistance;
	}
	int Sensors::GetFloorLeftValue()
	{
		return m_floorLeftValue;
	}
	int Sensors::GetFloorCenterValue()
	{
		return m_floorCenterValue;
	}
	int Sensors::GetFloorRightValue()
	{
		return m_floorRightValue;
	}

	bool LcdDisplay::Init()
	{
		if (!m_ledFrontLeft.Init(Basic::GPIO_PIN_LCD_LED_FRONT_LEFT, true) ||
			!m_ledFrontRight.Init(Basic::GPIO_PIN_LCD_LED_FRONT_RIGHT, true) ||
			// !m_ledBackLeft.Init(Basic::GPIO_PIN_LCD_LED_BACK_LEFT, true) ||	// Do not use when using picar
			!m_ledBackRight.Init(Basic::GPIO_PIN_LCD_LED_BACK_RIGHT, true))
		{
			printf("Fail to init LCD LED GPIO\n");
			return false;
		}
		if (!EP0152::LCD_I2C::Init())
		{
			printf("Fail to init LCD Display\n");
			return false;
		}

		m_isStop = false;
		m_ledFrontLeft.SetOutput(false);
		m_ledFrontRight.SetOutput(false);
		// m_ledBackLeft.SetOutput(false);		// Do not use when using picar
		m_ledBackRight.SetOutput(false);
		m_cpuTemp = 0.f;
		m_throttleState = 0;
		m_updateThread = thread(&LcdDisplay::updateThreadFunc, this);

		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for LCD\n");
			return false;
		}
		m_pubSubClient.ChangePubTopic("STATE_LCD_DISPLAY");
		m_pubThread = thread(&LcdDisplay::pubThreadFunc, this);

		return true;
	}
	void LcdDisplay::Release()
	{
		m_isStop = true;
		m_updateThread.join();
		m_pubThread.join();
	}

	void LcdDisplay::updateThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Lcd Updater Thread");

		chrono::steady_clock::time_point start;
		Mat displayImg = Mat::zeros(GetImageSize(), CV_8U);
		char tempStrBuf[512];
		char throStrBuf[512];

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			displayImg.setTo(0);

			float temp = -1.f;
			int thro = -1;

			FILE *tempFile = popen("vcgencmd measure_temp", "r");
			FILE *throFile = popen("vcgencmd get_throttled", "r");
			if (tempFile)
			{
				string tempStr = "";
				while (fgets(tempStrBuf, 512, tempFile) != NULL)
					tempStr += tempStrBuf;
				pclose(tempFile);

				try
				{
					sscanf(tempStr.c_str(), "temp=%f\'C", &temp);
				}
				catch (...)
				{
				}
			}
			if (throFile)
			{
				string tempStr = "";
				while (fgets(throStrBuf, 512, throFile) != NULL)
					tempStr += throStrBuf;
				pclose(throFile);

				try
				{
					sscanf(tempStr.c_str(), "throttled=%x", &thro);
				}
				catch (...)
				{
				}
			}

			{
				unique_lock lock(m_syncMutex);
				if (temp > 0.f)
					m_cpuTemp = temp;
				if (thro >= 0)
					m_throttleState = thro;

				m_ledFrontLeft.SetOutput(temp > 60.f);
				m_ledFrontRight.SetOutput(m_throttleState & 0x12);
				sprintf(tempStrBuf, "Temp : %.01f", m_cpuTemp);
				sprintf(throStrBuf, "State : %X", m_throttleState);
			}

			putText(displayImg, throStrBuf, Point(0, 14), FONT_HERSHEY_DUPLEX, 0.4, 1);
			putText(displayImg, tempStrBuf, Point(0, 30), FONT_HERSHEY_DUPLEX, 0.4, 1);

			SetImage(displayImg);

			this_thread::sleep_for(LCD_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
	void LcdDisplay::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Lcd Publisher Thread");

		chrono::steady_clock::time_point start;
		double cpuTemp;
		int throttleState;

		while (!m_isStop)
		{
			{
				shared_lock lock(m_syncMutex);
				start = chrono::steady_clock::now();
				cpuTemp = m_cpuTemp;
				throttleState = m_throttleState;
			}

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp(cpuTemp);
				pubMsg.addtyp(throttleState);
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in LCD\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}

	bool LidarSensor::Init()
	{
		if (!LD06::Lidar::Init())
		{
			printf("Fail to init Lidar sensor\n");
			return false;
		}

		m_isStop = false;
		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for Lidar sensor\n");
			return false;
		}
		m_pubSubClient.ChangePubTopic("STATE_LIDAR_SENSOR");
		m_pubThread = thread(&LidarSensor::pubThreadFunc, this);

		return true;
	}
	void LidarSensor::Release()
	{
		LD06::Lidar::Release();
		m_isStop = true;
		m_pubThread.join();
	}

	void LidarSensor::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Lidar Publisher Thread");

		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			vector<DataType> data = GetData();

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp<int>(data.size());
				for (size_t i = 0; i < data.size(); i++)
				{
					pubMsg.addtyp(data[i].Degree);
					pubMsg.addtyp(data[i].Distance);
					pubMsg.addtyp(data[i].Intensity);
				}
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in Lidar sensor\n");
			}
			this_thread::sleep_for(LIDAR_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}

	bool CameraSensor::Init(int w, int h, int bufSize, int frameRate)
	{
		m_isStop = false;

		if (!DirectCamera::Init(w, h, bufSize, frameRate))
			return false;

		if (!m_pubSubClient.Init(PROXY_XSUB_STR, PROXY_XPUB_STR))
		{
			printf("Fail to init ZMQ for camera sensor\n");
			return false;
		}
		m_pubSubClient.ChangePubTopic("STATE_CAMERA_SENSOR");
		m_pubThread = thread(&CameraSensor::pubThreadFunc, this);

		return true;
	}
	void CameraSensor::Release()
	{
		m_isStop = true;
		m_pubThread.join();
	}

	void CameraSensor::pubRawEncodedImage()
	{
		try
		{
			Camera::ImageInfo leftImageInfo, rightImageInfo;
			if (!GetFrame(leftImageInfo, rightImageInfo))
				return;

			int w = leftImageInfo.Image.cols;
			int h = leftImageInfo.Image.rows;
			int ch = leftImageInfo.Image.channels();

			zmq::multipart_t msg;
			msg.addtyp(w);
			msg.addtyp(h);
			msg.addtyp(ch);
			msg.addmem(leftImageInfo.Image.data, w * h * ch);
			msg.addmem(rightImageInfo.Image.data, w * h * ch);
			m_pubSubClient.PublishMessage(msg);
		}
		catch (...)
		{
			printf("Fail to publish massage in CameraSensor\n");
		}
	}
	void CameraSensor::pubJpgEncodedImage()
	{
		try
		{
			Camera::ImageInfo leftImageInfo, rightImageInfo;
			if (!GetFrame(leftImageInfo, rightImageInfo))
				return;

			vector<int> encodeParas;
			encodeParas.push_back(cv::IMWRITE_JPEG_QUALITY);
			encodeParas.push_back(90); // 0...100 (higher is better)

			vector<uchar> img_left_encoded, img_right_encoded;
			cv::imencode(".jpg", leftImageInfo.Image, img_left_encoded, encodeParas);
			cv::imencode(".jpg", rightImageInfo.Image, img_right_encoded, encodeParas);

			zmq::multipart_t msg;
			msg.addmem(img_left_encoded.data(), img_left_encoded.size());
			msg.addmem(img_right_encoded.data(), img_right_encoded.size());
			m_pubSubClient.PublishMessage(msg);
		}
		catch (...)
		{
			printf("Fail to publish massage in CameraSensor\n");
		}
	}
	void CameraSensor::pubPngEncodedImage()
	{
		try
		{
			Camera::ImageInfo leftImageInfo, rightImageInfo;
			if (!GetFrame(leftImageInfo, rightImageInfo))
				return;

			vector<int> encodeParas;
			encodeParas.push_back(cv::IMWRITE_PNG_COMPRESSION);
			encodeParas.push_back(1); // 0~7

			vector<uchar> img_left_encoded, img_right_encoded;
			cv::imencode(".jpg", leftImageInfo.Image, img_left_encoded, encodeParas);
			cv::imencode(".jpg", rightImageInfo.Image, img_right_encoded, encodeParas);

			zmq::multipart_t msg;
			msg.addmem(img_left_encoded.data(), img_left_encoded.size());
			msg.addmem(img_right_encoded.data(), img_right_encoded.size());
			m_pubSubClient.PublishMessage(msg);
		}
		catch (...)
		{
			printf("Fail to publish massage in CameraSensor\n");
		}
	}
	void CameraSensor::pubThreadFunc()
	{
		pthread_setname_np(pthread_self(), "Camera Sensor Publisher Thread");

		chrono::steady_clock::time_point start;
		const chrono::milliseconds CAMERA_DALTA_DUATION = chrono::milliseconds((int)round(1000.0 / GetFrameRate()));

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();

			pubJpgEncodedImage();

			this_thread::sleep_for(CAMERA_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
}
