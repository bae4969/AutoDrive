#include "Hardware.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>

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

	static const chrono::milliseconds MOTION_DALTA_DUATION = chrono::milliseconds(33);
	static const chrono::milliseconds CAMERA_DALTA_DUATION = chrono::milliseconds(33);

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

		m_rearSyncMutex.lock();
		bool isGood =
			m_leftDir.SetOutput(!isForward) &&
			m_rightDir.SetOutput(isForward) &&
			m_leftMotor.SetValue(absValue) &&
			m_rightMotor.SetValue(absValue);
		m_rearSyncMutex.unlock();

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
			start = chrono::steady_clock::now();

			m_updateMutex.lock();
			curRearValue = GetRearValue();
			curSteerDegree = GetSteerDegree();
			tarRearValue = m_targetRearValue;
			tarSteerDegree = m_targetSteerDegree;
			delDiffRearValue = m_deltaRearValue;
			delDiffSteerDegree = m_deltaSteerDegree;
			m_updateMutex.unlock();

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
		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			m_updateMutex.lock();
			int curRearValue = GetRearValue();
			int tarRearValue = m_targetRearValue;
			int delDiffRearValue = m_deltaRearValue;
			float curSteerDegree = GetSteerDegree();
			float tarSteerDegree = m_targetSteerDegree;
			float delDiffSteerDegree = m_deltaSteerDegree;
			m_updateMutex.unlock();

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
		m_updateMutex.lock();
		m_targetRearValue = 0;
		m_updateMutex.unlock();
		return updateRearValue(m_targetRearValue);
	}
	void MoveMotor::SetRearValue(int value)
	{
		if (value > REAR_MAX_VALUE)
			value = REAR_MAX_VALUE;
		if (value < REAR_MIN_VALUE)
			value = REAR_MIN_VALUE;

		m_updateMutex.lock();
		m_targetRearValue = value;
		m_updateMutex.unlock();
	}
	void MoveMotor::SetRearSpeed(int valuePerSecond)
	{
		m_updateMutex.lock();
		m_deltaRearValue = abs(round(valuePerSecond * MOTION_DALTA_DUATION.count() / 1000.0));
		m_updateMutex.unlock();
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

		m_updateMutex.lock();
		m_targetSteerDegree = degree;
		m_updateMutex.unlock();
	}
	void MoveMotor::SetSteerSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaSteerDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
		m_updateMutex.unlock();
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
			start = chrono::steady_clock::now();

			m_updateMutex.lock();
			curPitchDegree = GetPitchDegree();
			curYawDegree = GetYawDegree();
			tarPitchDegree = m_targetPitchDegree;
			tarYawDegree = m_targetYawDegree;
			delDiffPitchDegree = m_deltaPitchDegree;
			delDiffYawDegree = m_deltaYawDegree;
			m_updateMutex.unlock();

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
		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			m_updateMutex.lock();
			float curPitchDegree = GetPitchDegree();
			float tarPitchDegree = m_targetPitchDegree;
			float delDiffPitchDegree = m_deltaPitchDegree;
			float curYawDegree = GetYawDegree();
			float tarYawDegree = m_targetYawDegree;
			float delDiffYawDegree = m_deltaYawDegree;
			m_updateMutex.unlock();

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

		m_updateMutex.lock();
		m_targetPitchDegree = degree;
		m_updateMutex.unlock();
	}
	void CameraMotor::SetPitchSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaPitchDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
		m_updateMutex.unlock();
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

		m_updateMutex.lock();
		m_targetYawDegree = degree;
		m_updateMutex.unlock();
	}
	void CameraMotor::SetYawSpeed(float degreePerSecond)
	{
		m_updateMutex.lock();
		m_deltaYawDegree = abs(degreePerSecond * MOTION_DALTA_DUATION.count() / 1000.0f);
		m_updateMutex.unlock();
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

		m_floorSyncMutex.lock();
		m_floorLeftValue = t_v1;
		m_floorCenterValue = t_v2;
		m_floorRightValue = t_v3;
		m_floorSyncMutex.unlock();
	}
	void Sensors::updateThreadFunc()
	{
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
		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			m_floorSyncMutex.lock();
			double sonicValue = m_sonicDistance;
			int floorLeftValue = m_floorLeftValue;
			int floorCenterValue = m_floorCenterValue;
			int floorRightValue = m_floorRightValue;
			m_floorSyncMutex.unlock();

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

			m_syncMutex.lock();
			if (temp > 0.f)
				m_cpuTemp = temp;
			if (thro >= 0)
				m_throttleState = thro;

			m_ledFrontLeft.SetOutput(temp > 60.f);
			m_ledFrontRight.SetOutput(m_throttleState & 0x12);
			sprintf(tempStrBuf, "Temp : %.01f", m_cpuTemp);
			sprintf(throStrBuf, "State : %X", m_throttleState);
			m_syncMutex.unlock();

			putText(displayImg, throStrBuf, Point(0, 14), FONT_HERSHEY_DUPLEX, 0.4, 1);
			putText(displayImg, tempStrBuf, Point(0, 30), FONT_HERSHEY_DUPLEX, 0.4, 1);

			SetImage(displayImg);

			this_thread::sleep_for(1000ms - (chrono::steady_clock::now() - start));
		}
	}
	void LcdDisplay::pubThreadFunc()
	{
		chrono::steady_clock::time_point start;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			m_syncMutex.lock();

			m_syncMutex.unlock();

			try
			{
				zmq::multipart_t pubMsg;
				pubMsg.addtyp(m_cpuTemp);
				pubMsg.addtyp(m_throttleState);
				m_pubSubClient.PublishMessage(pubMsg);
			}
			catch (...)
			{
				printf("Fail to publish massage in LCD\n");
			}
			this_thread::sleep_for(MOTION_DALTA_DUATION - (chrono::steady_clock::now() - start));
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
			Camera::ImageInfo imageInfo;
			if (!GetFrame(imageInfo))
				return;

			int w = imageInfo.Image.cols;
			int h = imageInfo.Image.rows;
			int ch = imageInfo.Image.channels();

			zmq::multipart_t msg;
			msg.addtyp(w);
			msg.addtyp(h);
			msg.addtyp(ch);
			msg.addmem(imageInfo.Image.data, w * h * ch);
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
			Camera::ImageInfo imageInfo;
			if (!GetFrame(imageInfo))
				return;

			vector<int> encodeParas;
			encodeParas.push_back(cv::IMWRITE_JPEG_QUALITY);
			encodeParas.push_back(90); // 0...100 (higher is better)

			vector<uchar> img_encoded;
			cv::imencode(".jpg", imageInfo.Image, img_encoded, encodeParas);

			zmq::multipart_t msg;
			msg.addmem(img_encoded.data(), img_encoded.size());
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
			Camera::ImageInfo imageInfo;
			if (!GetFrame(imageInfo))
				return;

			vector<int> encodeParas;
			encodeParas.push_back(cv::IMWRITE_PNG_COMPRESSION);
			encodeParas.push_back(1); // 0~7

			vector<uchar> img_encoded;
			cv::imencode(".png", imageInfo.Image, img_encoded, encodeParas);

			zmq::multipart_t msg;
			msg.addmem(img_encoded.data(), img_encoded.size());
			m_pubSubClient.PublishMessage(msg);
		}
		catch (...)
		{
			printf("Fail to publish massage in CameraSensor\n");
		}
	}
	void CameraSensor::pubThreadFunc()
	{
		chrono::steady_clock::time_point start;
		vector<shared_ptr<thread>> threadPool(3, NULL);
		int threadIdx = -1;

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();

			threadIdx++;
			if (threadIdx > 2)
				threadIdx = 0;

			if (threadPool[threadIdx])
				threadPool[threadIdx]->join();

			threadPool[threadIdx] = make_shared<thread>(&CameraSensor::pubJpgEncodedImage, this);

			this_thread::sleep_for(CAMERA_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}

		for (int i = 0; i < threadPool.size(); i++)
			if (threadPool[i])
				threadPool[i]->join();
	}
}
