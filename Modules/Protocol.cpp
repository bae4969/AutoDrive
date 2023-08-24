#include "Protocol.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <mutex>

using namespace std;

namespace Protocol
{
	static bool IsSetGPIO = false;
	static int I2C_FD = -1;
	const int I2C_ADDRESS = 0x14;
	static mutex i2cWriteMutex;

	bool InitProtocol()
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

		I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", I2C_ADDRESS);
		if (I2C_FD < 0)
		{
			printf("Fail to init I2C\n");
			return false;
		}

		if (wiringPiI2CWrite(I2C_FD, 0x2C) < 0)
		{
			printf("Fail to init I2C again\n");
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

	I2C::I2C()
	{
		m_pulseWidth = 0;
		m_frequency = 50;
		m_prescaler = 10;
		m_period = 4095;
	}
	ushort I2C::convertBig2Little(ushort val)
	{
		ushort val_h = val >> 8;
		ushort val_l = val & 0xff;
		return (val_l << 8) + val_h;
	}
	bool I2C::Init(int channel, ushort prescaler, ushort period)
	{
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return false;
		}

		m_channel = channel;
		m_group = channel / 4;
		if (!SetFrequency(50) ||
			!SetPrescaler(prescaler) ||
			!SetPeriod(period))
		{
			printf("Fail to init I2C %d\n", m_channel);
			return false;
		}

		return true;
	}
	bool I2C::SetPulseWidth(ushort value)
	{
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return false;
		}

		int reg = PULSE_WIDTH_REG_OFFSET + m_channel;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write I2C %d pulse width\n", reg);
			return false;
		}

		m_pulseWidth = value;
		return true;
	}
	bool I2C::SetFrequency(ushort value)
	{
		m_frequency = value;
		return true;

		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
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
	bool I2C::SetPrescaler(ushort value)
	{
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return false;
		}

		int reg = PRESCALER_REG_OFFSET + m_group;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write I2C %d prescaler\n", reg);
			return false;
		}

		m_prescaler = value;
		return true;
	}
	bool I2C::SetPeriod(ushort value)
	{
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return false;
		}

		int reg = PERIOD_REG_OFFSET + m_group;
		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, convertBig2Little(value));
		i2cWriteMutex.unlock();
		if (ret < 0)
		{
			printf("Fail to write I2C %d period\n", reg);
			return false;
		}

		m_period = value;
		return true;
	}
	ushort I2C::GetPulseWidth()
	{
		return m_pulseWidth;
	}
	ushort I2C::GetFrequency()
	{
		return m_frequency;
	}
	ushort I2C::GetPrescaler()
	{
		return m_prescaler;
	}
	ushort I2C::GetPeriod()
	{
		return m_period;
	}

	PWMMotor::PWMMotor()
	{
		m_curValue = 0;
	}
	bool PWMMotor::Init(int channel)
	{
		return I2C::Init(channel, 9, 4095) &&
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

		if (!I2C::Init(channel, 350, 4094))
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
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return false;
		}

		m_channel = 0x10 | (7 - channel);
		return true;
	}
	int ADC::GetValue()
	{
		if (I2C_FD < 0)
		{
			printf("I2C FD is not setted\n");
			return -1;
		}

		i2cWriteMutex.lock();
		int ret = wiringPiI2CWriteReg16(I2C_FD, m_channel, 0);
		int first = wiringPiI2CRead(I2C_FD);
		int second = wiringPiI2CRead(I2C_FD);
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
		m_subSocket->connect(subConnStr);
		m_pubSocket->connect(pubConnStr);

		return true;
	}
	PubSubClient::~PubSubClient()
	{
		m_subSocket->close();
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
	bool PubSubClient::SubscribeMessage(zmq::multipart_t& msg)
	{
		return msg.recv(*m_subSocket);
	}

	bool PubSubServer::Init(vector<string> xPubConnStrs, vector<string> xSubConnStrs)
	{
		m_zmqContext = make_shared<zmq::context_t>(2);
		m_xSubSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::xsub);
		m_xPubSocket = make_shared<zmq::socket_t>(*m_zmqContext, zmq::socket_type::xpub);

		for (string xSubConnStr : xSubConnStrs)
			m_xSubSocket->bind(xSubConnStr);
		for (string xPubConnStr : xPubConnStrs)
			m_xPubSocket->bind(xPubConnStr);

		m_subThread = thread(
			[](shared_ptr<zmq::socket_t> sub, shared_ptr<zmq::socket_t> pub)
			{
				zmq::proxy(*sub, *pub);
			},
			m_xSubSocket,
			m_xPubSocket);

		return true;
	}
	PubSubServer::~PubSubServer()
	{
		m_xSubSocket->close();
		m_xPubSocket->close();
		m_subThread.join();
	}
}
