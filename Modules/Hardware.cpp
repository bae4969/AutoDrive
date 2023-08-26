#include "Hardware.h"
#include <iostream>
#include <math.h>

#define REAR_MIN_VALUE -2000
#define REAR_MAX_VALUE 2000
#define STEER_MIN_DEGREE -30
#define STEER_MAX_DEGREE 30
#define CAMERA_PITCH_MIN_DEGREE -30
#define CAMERA_PITCH_MAX_DEGREE 50
#define CAMERA_YAW_MIN_DEGREE -60
#define CAMERA_YAW_MAX_DEGREE 60

using namespace std;

namespace Hardware
{
	const std::chrono::milliseconds MOTION_DALTA_DUATION = std::chrono::milliseconds(33);
	const std::chrono::milliseconds CAMERA_DALTA_DUATION = std::chrono::milliseconds(50);

	bool MoveMotor::Init(float defaultSteerAngle)
	{
		if (!m_leftDir.Init(Protocol::GPIO_PIN_REAR_LEFT_DIRECTION, true))
		{
			printf("Fail to init rear left direction GPIO\n");
			return false;
		}
		if (!m_rightDir.Init(Protocol::GPIO_PIN_REAR_RIGHT_DIRECTION, true))
		{
			printf("Fail to init rear right direction GPIO\n");
			return false;
		}
		if (!m_leftMotor.Init(Protocol::I2C_CHANNEL_REAR_LEFT))
		{
			printf("Fail to init rear left PWM motor\n");
			return false;
		}
		if (!m_rightMotor.Init(Protocol::I2C_CHANNEL_REAR_RIGHT))
		{
			printf("Fail to init rear right PWM motor\n");
			return false;
		}
		if (!m_steerMotor.Init(Protocol::I2C_CHANNEL_FRONT_STEER, defaultSteerAngle))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetRearSpeed(500);
		SetRearValue(0);
		SetSteerSpeed(50.0f);
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
					throw std::exception();

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
		if (!m_pitchMotor.Init(Protocol::I2C_CHANNEL_CAMERA_TILT, -defaultPitchDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		if (!m_yawMotor.Init(Protocol::I2C_CHANNEL_CAMERA_YAW, -defaultYawDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}

		m_isStop = false;
		SetPitchSpeed(50.0f);
		SetPitchDegree(0.0f);
		SetYawSpeed(50.0f);
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
					throw std::exception();

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
		if (!m_tring.Init(Protocol::GPIO_PIN_SONIC_TRING, true) ||
			!m_echo.Init(Protocol::GPIO_PIN_SONIC_ECHO, false))
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

	void CameraSensor::pubThreadFunc()
	{
		chrono::steady_clock::time_point start;
		Camera::ImageInfo imageInfo;
		cv::Size imgSize = GetSize();
		int w = imgSize.width;
		int h = imgSize.height;
		int ch = 3;
		size_t totSize = imgSize.area() * 3;

		// string encodeType = ".png";
		// vector<int> encodeParas;
		// encodeParas.push_back(cv::IMWRITE_PNG_COMPRESSION);
		// encodeParas.push_back(1);	// 0~7

		// string encodeType = ".png";
		// vector<int> encodeParas;
		// encodeParas.push_back(cv::IMWRITE_JPEG_QUALITY);
		// encodeParas.push_back(90);     // 0...100 (higher is better)

		while (!m_isStop)
		{
			start = chrono::steady_clock::now();
			try
			{
				if (GetFrame(imageInfo))
				{
					cv::Mat &img = imageInfo.Image;

					// vector<uchar> img_encoded;
					// cv::imencode(encodeType, img, img_encoded, encodeParas);

					zmq::multipart_t msg;
					msg.addtyp(w);
					msg.addtyp(h);
					msg.addtyp(ch);
					msg.addmem(img.data, totSize);
					// msg.addmem(img_encoded.data(), img_encoded.size());
					m_pubSubClient.PublishMessage(msg);
				}
			}
			catch (...)
			{
				printf("Fail to publish massage in CameraSensor\n");
			}
			this_thread::sleep_for(CAMERA_DALTA_DUATION - (chrono::steady_clock::now() - start));
		}
	}
}
