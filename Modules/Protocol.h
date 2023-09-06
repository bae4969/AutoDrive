#pragma once
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <atomic>
#include <thread>
#include <boost/shared_ptr.hpp>

namespace Protocol
{
	typedef unsigned short ushort;
	enum GPIO_PIN
	{
		GPIO_PIN_REAR_LEFT_DIRECTION = 4,
		GPIO_PIN_REAR_RIGHT_DIRECTION = 5,
		GPIO_PIN_SONIC_TRING = 2,
		GPIO_PIN_SONIC_ECHO = 3,
		GPIO_PIN_MCU_RESET = 21,
		GPIO_PIN_SWITCH = 6,
		GPIO_PIN_CAR_LED = 25,
		GPIO_PIN_LCD_LED_FRONT_LEFT = 23,
		GPIO_PIN_LCD_LED_FRONT_RIGHT = 24,
		GPIO_PIN_LCD_LED_BACK_LEFT = 21,	// duplicate pin num with reset
		GPIO_PIN_LCD_LED_BACK_RIGHT = 22,
	};
	enum CAR_I2C_CHANNEL
	{
		CAR_I2C_CHANNEL_CAMERA_YAW = 0,
		CAR_I2C_CHANNEL_CAMERA_TILT = 1,
		CAR_I2C_CHANNEL_FRONT_STEER = 2,
		CAR_I2C_CHANNEL_REAR_LEFT = 13,
		CAR_I2C_CHANNEL_REAR_RIGHT = 12,
	};

	bool InitCarProtocol();

	class GPIO
	{
	private:
		int m_pinIdx = -1;
		std::atomic<bool> m_isOut;
		std::atomic<int> m_isHigh;

	public:
		bool Init(int pinIdx, bool isOut);
		bool ChangeMode(bool isOut);
		bool SetOutput(bool isHigh);
		int GetOutput();
		int GetInput();
	};

	class CAR_I2C
	{
	private:
		const int PULSE_WIDTH_REG_OFFSET = 0x20;
		const int FREQUENCY_REG_OFFSET = 0x30;
		const int PRESCALER_REG_OFFSET = 0x40;
		const int PERIOD_REG_OFFSET = 0x44;

		int m_channel = -1;
		int m_group = -1;
		std::atomic<ushort> m_pulseWidth;
		std::atomic<ushort> m_frequency;
		std::atomic<ushort> m_prescaler;
		std::atomic<ushort> m_period;

		ushort convertBig2Little(ushort value);

	public:
		CAR_I2C();
		bool Init(int channel, ushort prescaler, ushort period);
		bool SetPulseWidth(ushort value);
		bool SetFrequency(ushort value);
		bool SetPrescaler(ushort value);
		bool SetPeriod(ushort value);
		ushort GetPulseWidth();
		ushort GetFrequency();
		ushort GetPrescaler();
		ushort GetPeriod();
	};

	class LCD_I2C
	{
		bool m_isInit;
		int m_width;
		int m_height;
		int m_page;
		int m_line;
		cv::Mat m_img8;

	public:
		LCD_I2C();
		~LCD_I2C();

		bool Init();
		bool Reset();
		bool SetImage(cv::Mat img8);
		cv::Mat GetImage();
		cv::Size GetImageSize();
	};

	class PWMMotor : public CAR_I2C
	{
	private:
		std::atomic<ushort> m_curValue;

	public:
		PWMMotor();
		bool Init(int channel);
		bool SetValue(ushort value);
		ushort GetValue();
	};

	class ServoMotor : public CAR_I2C
	{
	private:
		float m_defaultDegree;
		float m_curDegree;

		ushort convertDegreeToPulseWidth(float degree);

	public:
		ServoMotor();
		bool Init(int channel, float defaultDegree, bool isSetZero = true);
		bool SetDegree(float degree);
		float GetDegree();
	};

	class ADC
	{
	private:
		int m_channel = -1;

	public:
		bool Init(int channel);
		int GetValue();
	};

	class PubSubClient
	{
		std::shared_ptr<zmq::context_t> m_zmqContext = NULL;
		std::shared_ptr<zmq::socket_t> m_subSocket = NULL;
		std::shared_ptr<zmq::socket_t> m_pubSocket = NULL;
		std::string m_pubTopic;

	public:
		bool Init(std::string pubConnStr, std::string subConnStr);
		~PubSubClient();

		void ChangePubTopic(std::string topic);
		void PublishMessage(zmq::multipart_t &msg);
		void AddSubTopic(std::string topic);
		void RemoveSubTopic(std::string topic);
		bool SubscribeMessage(zmq::multipart_t &msg);
	};
	class PubSubServer
	{
		std::thread m_subThread;
		std::shared_ptr<zmq::context_t> m_zmqContext = NULL;
		std::shared_ptr<zmq::socket_t> m_xSubSocket = NULL;
		std::shared_ptr<zmq::socket_t> m_xPubSocket = NULL;

	public:
		bool Init(std::vector<std::string> xPubConnStrs, std::vector<std::string> xSubConnStrs);
		~PubSubServer();
	};

}
