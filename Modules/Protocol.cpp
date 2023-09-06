#include "Protocol.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <mutex>

namespace Protocol
{
	using namespace std;
	using namespace cv;

	static bool IsSetGPIO = false;
	static int CAR_I2C_FD = -1;
	static int LCD_I2C_FD = -1;
	const int CAR_I2C_ADDRESS = 0x14;
	const int LCD_I2C_ADDRESS = 0x3C;
	static mutex i2cWriteMutex;

	bool InitCarProtocol()
	{
		IsSetGPIO = wiringPiSetup() == 0;
		if (!IsSetGPIO)
		{
			printf("Fail to init GPIO\n");
			return false;
		}

		GPIO rstMCU;
		rstMCU.Init(GPIO_PIN_MCU_RESET, true);
		rstMCU.SetOutput(false);
		this_thread::sleep_for(chrono::milliseconds(100));
		rstMCU.SetOutput(true);
		this_thread::sleep_for(chrono::milliseconds(300));

		CAR_I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", CAR_I2C_ADDRESS);
		if (CAR_I2C_FD < 0)
		{
			printf("Fail to init CAR I2C FD\n");
			return false;
		}

		if (wiringPiI2CWrite(CAR_I2C_FD, 0x2C) < 0)
		{
			printf("Fail to init CAR I2C\n");
			return false;
		}

		return true;
	}

	bool GPIO::Init(int pinIdx, bool isOut)
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not setted\n");
			return false;
		}

		m_pinIdx = pinIdx;
		m_isOut = isOut;
		pinMode(m_pinIdx, m_isOut);

		return true;
	}
	bool GPIO::ChangeMode(bool isOut)
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not setted\n");
			return false;
		}
		if (m_pinIdx < 0)
		{
			printf("This GPIO %d is not init\n", m_pinIdx);
			return false;
		}

		m_isOut = isOut;
		pinMode(m_pinIdx, m_isOut);

		return true;
	}
	bool GPIO::SetOutput(bool isHigh)
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not setted\n");
			return false;
		}
		if (!m_isOut)
		{
			printf("This GPIO %d is not out mode\n", m_pinIdx);
			return false;
		}

		m_isHigh = isHigh;
		digitalWrite(m_pinIdx, m_isHigh);

		return true;
	}
	int GPIO::GetOutput()
	{
		if (!m_isOut)
		{
			printf("This GPIO %d is not out mode\n", m_pinIdx);
			return -1;
		}

		return m_isHigh;
	}
	int GPIO::GetInput()
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not setted\n");
			return -1;
		}
		if (m_isOut)
		{
			printf("This GPIO %d is not in mode\n", m_pinIdx);
			return -1;
		}

		m_isHigh = digitalRead(m_pinIdx);

		return m_isHigh;
	}

	LCD_I2C::LCD_I2C()
	{
		m_isInit = false;
		m_width = 128;
		m_height = 32;
		m_page = 8;
		m_line = 4;
		m_img8 = Mat::zeros(m_height, m_width, CV_8U);
	}
	LCD_I2C::~LCD_I2C()
	{
		if (!m_isInit)
			return;

		wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xAE); // SSD1306_DISPLAY_OFF
	}
	bool LCD_I2C::Init()
	{
		LCD_I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", LCD_I2C_ADDRESS);
		if (LCD_I2C_FD < 0)
		{
			printf("Fail to init LCD I2C FD\n");
			return false;
		}

		if (wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xAE) < 0 ||		   // SSD1306_DISPLAY_OFF
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xD5) < 0 ||		   // SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x80) < 0 ||		   // 0x80
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xA8) < 0 ||		   // SSD1306_SET_MULTIPLEX_RATIO
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x1F) < 0 ||		   // 0x1F
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xD3) < 0 ||		   // SSD1306_SET_DISPLAY_OFFSET
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x00) < 0 ||		   // 0x00
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x40 | 0x00) < 0 ||	  // SSD1306_SET_START_LINE
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x8D) < 0 ||		   // SSD1306_CHARGE_PUMP
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x14) < 0 ||		   // 0x14
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x20) < 0 ||		   // SSD1306_MEMORY_ADDR_MODE
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x00) < 0 ||		   // 0x00
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xA0 | 0x01) < 0 || // SSD1306_SET_SEGMENT_REMAP
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xC8) < 0 ||		   // SSD1306_COM_SCAN_DIR_DEC
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xDA) < 0 ||		   // SSD1306_SET_COM_PINS
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x02) < 0 ||		   // 0x02
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x81) < 0 ||		   // SSD1306_SET_CONTRAST_CONTROL
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x8F) < 0 ||		   // 0x8F
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xD9) < 0 ||		   // SSD1306_SET_PRECHARGE_PERIOD
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xF1) < 0 ||		   // 0xF1
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xDB) < 0 ||		   // SSD1306_SET_VCOM_DESELECT
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x40) < 0 ||		   // 0x40
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xA4) < 0 ||		   // SSD1306_DISPLAY_ALL_ON_RESUME
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xA6) < 0 ||		   // SSD1306_NORMAL_DISPLAY
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x2E) < 0 ||		   // SSD1306_DEACTIVATE_SCROLL
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xAF) < 0)		   // SSD1306_DISPLAY_ON
		{
			printf("Fail to init LCD I2C\n");
			return false;
		}

		m_isInit = true;

		return true;
	}
	bool LCD_I2C::Reset()
	{
		return SetImage(Mat::zeros(m_height, m_width, CV_8U));
	}
	bool LCD_I2C::SetImage(Mat img8)
	{
		if (m_img8.cols != img8.cols ||
			m_img8.rows != img8.rows ||
			m_img8.cols != img8.cols ||
			m_img8.cols != img8.cols)
		{
			printf("Image data type is not same in LCD I2C\n");
			return false;
		}

		memcpy(m_img8.data, img8.data, m_width * m_height);

		char totData;
		char data[8];
		char *imgPtr = (char *)m_img8.data;
		for (int lineIdx = 0; lineIdx < 4; lineIdx++)
			for (int x = 0; x < m_width; x++)
			{
				data[0] = (imgPtr[(lineIdx * 8 + 0) * m_width + x] > 0) << 0;
				data[1] = (imgPtr[(lineIdx * 8 + 1) * m_width + x] > 0) << 1;
				data[2] = (imgPtr[(lineIdx * 8 + 2) * m_width + x] > 0) << 2;
				data[3] = (imgPtr[(lineIdx * 8 + 3) * m_width + x] > 0) << 3;
				data[4] = (imgPtr[(lineIdx * 8 + 4) * m_width + x] > 0) << 4;
				data[5] = (imgPtr[(lineIdx * 8 + 5) * m_width + x] > 0) << 5;
				data[6] = (imgPtr[(lineIdx * 8 + 6) * m_width + x] > 0) << 6;
				data[7] = (imgPtr[(lineIdx * 8 + 7) * m_width + x] > 0) << 7;
				totData = data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7];

				if (wiringPiI2CWriteReg8(LCD_I2C_FD, 0x40, totData) < 0)
				{
					printf("Fail to write LCD I2C data\n");
					return false;
				}
			}

		return true;
	}
	Mat LCD_I2C::GetImage()
	{
		return m_img8.clone();
	}
	Size LCD_I2C::GetImageSize()
	{
		return Size(m_width, m_height);
	}

	CAR_I2C::CAR_I2C()
	{
		m_pulseWidth = 0;
		m_frequency = 50;
		m_prescaler = 10;
		m_period = 4095;
	}
	ushort CAR_I2C::convertBig2Little(ushort val)
	{
		ushort val_h = val >> 8;
		ushort val_l = val & 0xff;
		return (val_l << 8) + val_h;
	}
	bool CAR_I2C::Init(int channel, ushort prescaler, ushort period)
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		m_channel = channel;
		m_group = channel / 4;
		if (!SetFrequency(50) ||
			!SetPrescaler(prescaler) ||
			!SetPeriod(period))
		{
			printf("Fail to init CAR I2C %d\n", m_channel);
			return false;
		}

		return true;
	}
	bool CAR_I2C::SetPulseWidth(ushort value)
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		int reg = PULSE_WIDTH_REG_OFFSET + m_channel;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write CAR I2C %d pulse width\n", reg);
			return false;
		}

		m_pulseWidth = value;
		return true;
	}
	bool CAR_I2C::SetFrequency(ushort value)
	{
		m_frequency = value;
		return true;

		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		const int CLOCK = 72000000;
		int st = (int)sqrt(CLOCK / value);
		vector<int> prescalers(10, 0);
		vector<int> periods(10, 0);
		vector<int> accuracy(10, 0);
		for (int i = 0; i < 10; i++)
		{
			int t_prescaler = st + i;
			int t_period = int(CLOCK / value / t_prescaler);
			prescalers[i] = t_prescaler;
			periods[i] = t_period;
			accuracy[i] = abs((int)value - CLOCK / t_prescaler / t_period);
		}
		int minVal = INT32_MAX;
		int minIdx = -1;
		for (int i = 0; i < 10; i++)
		{
			if (minVal > accuracy[i])
			{
				minVal = accuracy[i];
				minIdx = i;
			}
		}
		if (minIdx < 0)
			return false;

		ushort prescaler = prescalers[minIdx] - 1;
		ushort period = periods[minIdx] - 1;

		if (!SetPrescaler(prescalers[minIdx]) ||
			!SetPeriod(periods[minIdx]))
			return false;

		m_frequency = value;
		return true;
	}
	bool CAR_I2C::SetPrescaler(ushort value)
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		int reg = PRESCALER_REG_OFFSET + m_group;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write CAR I2C %d prescaler\n", reg);
			return false;
		}

		m_prescaler = value;
		return true;
	}
	bool CAR_I2C::SetPeriod(ushort value)
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		int reg = PERIOD_REG_OFFSET + m_group;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write CAR I2C %d period\n", reg);
			return false;
		}

		m_period = value;
		return true;
	}
	ushort CAR_I2C::GetPulseWidth()
	{
		return m_pulseWidth;
	}
	ushort CAR_I2C::GetFrequency()
	{
		return m_frequency;
	}
	ushort CAR_I2C::GetPrescaler()
	{
		return m_prescaler;
	}
	ushort CAR_I2C::GetPeriod()
	{
		return m_period;
	}

	PWMMotor::PWMMotor()
	{
		m_curValue = 0;
	}
	bool PWMMotor::Init(int channel)
	{
		return CAR_I2C::Init(channel, 9, 4095) &&
			   SetPulseWidth(0);
	}
	bool PWMMotor::SetValue(ushort value)
	{
		if (!SetPulseWidth(value))
			return false;

		m_curValue = value;
		return true;
	}
	ushort PWMMotor::GetValue()
	{
		return m_curValue;
	}

	ServoMotor::ServoMotor()
	{
		m_defaultDegree = 0.0f;
		m_curDegree = 0.0f;
	}
	ushort ServoMotor::convertDegreeToPulseWidth(float degree)
	{
		degree += m_defaultDegree;
		const float in_min = -90.0f;
		const float in_max = 90.0f;
		const float out_min = 500.f;
		const float out_max = 2500.f;
		float highLevelTime = (degree - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		float power = highLevelTime / 20000.0f;
		float value32 = (int)round(power * 4094.0f);
		ushort value16;
		if (value32 < 0.0f)
			value16 = 0;
		else if (value32 > 65535.0f)
			value16 = 65535;
		else
			value16 = value32;

		return value16;
	}
	bool ServoMotor::Init(int channel, float defaultDegree, bool isSetZero)
	{
		m_defaultDegree = defaultDegree;

		if (!CAR_I2C::Init(channel, 350, 4094))
			return false;

		if (!isSetZero)
			return true;

		return SetDegree(0);
	}
	bool ServoMotor::SetDegree(float degree)
	{
		ushort pulseWidth = convertDegreeToPulseWidth(degree);
		if (!SetPulseWidth(pulseWidth))
			return false;

		m_curDegree = degree;
		return true;
	}
	float ServoMotor::GetDegree()
	{
		return m_curDegree;
	}

	bool ADC::Init(int channel)
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return false;
		}

		m_channel = 0x10 | (7 - channel);
		return true;
	}
	int ADC::GetValue()
	{
		if (CAR_I2C_FD < 0)
		{
			printf("CAR I2C FD is not setted\n");
			return -1;
		}

		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(CAR_I2C_FD, m_channel, 0);
		int first = wiringPiI2CRead(CAR_I2C_FD);
		int second = wiringPiI2CRead(CAR_I2C_FD);
		i2cWriteMutex.unlock();

		bool isGood = ret >= 0 || first >= 0 || second >= 0;

		return isGood ? (first << 8) + second : -1;
	}

	bool PubSubClient::Init(string pubConnStr, string subConnStr)
	{
		m_zmqContext = make_shared<zmq::context_t>(1);
		m_subSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::sub);
		m_pubSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::pub);
		m_pubTopic = "NOT DEFINED";

		m_subSocket->setsockopt(ZMQ_RCVTIMEO, 1);
		m_subSocket->setsockopt(ZMQ_RCVHWM, 20);
		m_pubSocket->setsockopt(ZMQ_SNDHWM, 20);

		m_subSocket->connect(subConnStr);
		m_pubSocket->connect(pubConnStr);

		return true;
	}
	PubSubClient::~PubSubClient()
	{
		if (m_subSocket)
			m_subSocket->close();
		if (m_pubSocket)
			m_pubSocket->close();
	}

	void PubSubClient::ChangePubTopic(std::string topic)
	{
		m_pubTopic = topic;
	}
	void PubSubClient::PublishMessage(zmq::multipart_t &msg)
	{
		msg.pushstr(m_pubTopic);
		msg.send(*m_pubSocket);
	}
	void PubSubClient::AddSubTopic(std::string topic)
	{
		m_subSocket->setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.length());
	}
	void PubSubClient::RemoveSubTopic(std::string topic)
	{
		m_subSocket->setsockopt(ZMQ_UNSUBSCRIBE, topic);
	}
	bool PubSubClient::SubscribeMessage(zmq::multipart_t &msg)
	{
		return msg.recv(*m_subSocket);
	}

	bool PubSubServer::Init(vector<string> xPubConnStrs, vector<string> xSubConnStrs)
	{
		m_zmqContext = make_shared<zmq::context_t>(3);
		m_xSubSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::xsub);
		m_xPubSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::xpub);
		m_xSubSocket->setsockopt(ZMQ_RCVHWM, 20);
		m_xPubSocket->setsockopt(ZMQ_SNDHWM, 20);

		for (string xSubConnStr : xSubConnStrs)
			m_xSubSocket->bind(xSubConnStr);
		for (string xPubConnStr : xPubConnStrs)
			m_xPubSocket->bind(xPubConnStr);

		m_subThread = thread(
			[](shared_ptr<zmq::socket_t> sub,
			   shared_ptr<zmq::socket_t> pub)
			{
				try
				{
					zmq::proxy(*sub, *pub);
				}
				catch (...)
				{
				}
			},
			m_xSubSocket,
			m_xPubSocket);

		return true;
	}
	PubSubServer::~PubSubServer()
	{
		if (m_xSubSocket)
			m_xSubSocket->close();
		if (m_xPubSocket)
			m_xPubSocket->close();
		m_subThread.join();
	}
}
