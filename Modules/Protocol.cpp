#include "Protocol.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <thread>

using namespace std;

namespace Protocol
{
	static bool IsSetGPIO = false;
	static int I2C_FD = -1;
	const int I2C_ADDRESS = 0x14;

	bool InitProtocol()
	{
		IsSetGPIO = wiringPiSetup() == 0;
		if (!IsSetGPIO)
		{
			printf("Fail to init GPIO\n");
			return false;
		}

		I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", I2C_ADDRESS);
		if (I2C_FD < 0)
		{
			printf("Fail to init I2C\n");
			return false;
		}

		try
		{
			if (wiringPiI2CWrite(I2C_FD, 0x2C) < 0)
			{
				printf("Fail to init I2C at once, so retry init\n");
				throw std::exception();
			}
		}
		catch (...)
		{
			I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", 0x15);
			I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", I2C_ADDRESS);
			if (wiringPiI2CWrite(I2C_FD, 0x2C) < 0)
			{
				printf("Fail to init I2C again\n");
				return false;
			}
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

	ushort I2C::ConvertBig2Little(ushort val)
	{
		ushort val_h = val >> 8;
		ushort val_l = val & 0xff;
		return (val_l << 8) + val_h;
	}
	bool I2C::Init(int channel, ushort prescaler, ushort period, ushort pulseWidth)
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
			!SetPeriod(period) ||
			!SetPulseWidth(pulseWidth))
		{
			printf("Fail to set init value\n");
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
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, ConvertBig2Little(value));
		if (ret < 0)
			return false;

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
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, ConvertBig2Little(value));
		if (ret < 0)
			return false;

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
		int ret = wiringPiI2CWriteReg16(I2C_FD, reg, ConvertBig2Little(value));
		if (ret < 0)
			return false;

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

	bool PWMMotor::SetMinMax(ushort min, ushort max)
	{
		if (min >= max)
		{
			printf("Min value is bigger than max value\n");
			return false;
		}
		ushort t_minValue = min;
		ushort t_maxValue = max;
		ushort t_curValue = m_curValue;
		if (t_curValue < t_minValue)
			t_curValue = t_minValue;
		if (t_curValue > t_maxValue)
			t_curValue = t_maxValue;

		float t_throttle = float(t_curValue - t_minValue) / float(t_maxValue - t_minValue);
		if (!SetPulseWidth(t_curValue))
		{
			printf("Fail to set min max value\n");
			return false;
		}

		m_minValue = t_minValue;
		m_maxValue = t_maxValue;
		m_curValue = t_curValue;
		m_throttle = t_throttle;
		return true;
	}
	bool PWMMotor::SetValue(ushort value)
	{
		if (value < m_minValue)
			value = m_minValue;
		if (value > m_maxValue)
			value = m_maxValue;
		float t_throttle = float(value - m_minValue) / float(m_maxValue - m_minValue);
		if (!SetPulseWidth(value))
		{
			printf("Fail to set min max value\n");
			return false;
		}

		m_curValue = value;
		m_throttle = t_throttle;
		return true;
	}
	bool PWMMotor::SetThrottle(float throttle)
	{
		if (throttle < 0.0f)
			throttle = 0.0f;
		if (throttle > 1.0f)
			throttle = 1.0f;
		ushort t_curValue = throttle * float(m_maxValue - m_minValue) + m_minValue;
		if (!SetPulseWidth(t_curValue))
		{
			printf("Fail to set min max value\n");
			return false;
		}

		m_curValue = t_curValue;
		m_throttle = throttle;
		return true;
	}
	ushort PWMMotor::GetMinValue()
	{
		return m_minValue;
	}
	ushort PWMMotor::GetMaxValue()
	{
		return m_maxValue;
	}
	ushort PWMMotor::GetCurrnetValue()
	{
		return m_curValue;
	}
	float PWMMotor::GetThrottle()
	{
		return m_throttle;
	}

	ushort ServoMotor::ConvertDegreeToPulseWidth(float degree)
	{
		degree += m_defaultDegree;
		const float in_min = -90.0f;
		const float in_max = 90.0f;
		const float out_min = 500.f;
		const float out_max = 2500.f;
		float highLevelTime = (degree - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		float power = highLevelTime / 20000.0f;
		float value32 = (int)round(power * GetPeriod());
		ushort value16;
		if (value32 < 0.0f)
			value16 = 0;
		else if (value32 > 65535.0f)
			value16 = 65535;
		else
			value16 = value32;

		return value16;
	}
	bool ServoMotor::Init(int channel, float defaultDegree, float minDegree, float maxDegree, bool isSetZero)
	{
		m_defaultDegree = defaultDegree;
		m_minDegree = minDegree;
		m_maxDegree = maxDegree;
		if (!I2C::Init(channel, 350, 4094, m_defaultDegree))
		{
			printf("Fail to init steer PWM motor\n");
			return false;
		}
		if (!isSetZero)
			return true;

		if (!SetDegreeWithTime(0))
		{
			printf("Fail to reset steer PWM motor\n");
			return false;
		}

		return true;
	}
	bool ServoMotor::SetDegreeWithTime(float degree, int millisecond)
	{
		if (degree > m_maxDegree)
			degree = m_maxDegree;
		if (degree < m_minDegree)
			degree = m_minDegree;

		int times = millisecond > 0 ? millisecond / DALTA_DUATION.count() : 1;
		float startDegree = m_curDegree;
		float endDegree = degree;
		float deltaDegree = (endDegree - startDegree) / times;
		for (int i = 0; i < times; i++)
		{
			auto exeStart = chrono::system_clock::now();
			float t_degree = startDegree + deltaDegree * i;

			ushort pulseWidth = ConvertDegreeToPulseWidth(t_degree);
			if (!SetPulseWidth(pulseWidth))
			{
				printf("Fail to set servo motor degree\n");
				return false;
			}

			auto exeEnd = chrono::system_clock::now();
			auto sleepTime = chrono::milliseconds(DALTA_DUATION) - (exeEnd - exeStart);
			m_curDegree = t_degree;
			this_thread::sleep_for(sleepTime);
		}

		ushort pulseWidth = ConvertDegreeToPulseWidth(degree);
		if (!SetPulseWidth(pulseWidth))
		{
			printf("Fail to set servo motor degree\n");
			return false;
		}

		m_curDegree = degree;
		return true;
	}
	bool ServoMotor::SetDegreeWithSpeed(float degree, float absDeltaDegree)
	{
		if (degree > m_maxDegree)
			degree = m_maxDegree;
		if (degree < m_minDegree)
			degree = m_minDegree;

		float startDegree = m_curDegree;
		float endDegree = degree;
		float deltaDegree = abs(absDeltaDegree);
		int times = abs(endDegree - startDegree) / deltaDegree;
		if (startDegree > endDegree)
			deltaDegree = -deltaDegree;
		for (int i = 0; i < times; i++)
		{
			auto exeStart = chrono::system_clock::now();
			float t_degree = startDegree + deltaDegree * i;

			ushort pulseWidth = ConvertDegreeToPulseWidth(t_degree);
			if (!SetPulseWidth(pulseWidth))
			{
				printf("Fail to set servo motor degree\n");
				return false;
			}

			auto exeEnd = chrono::system_clock::now();
			auto sleepTime = chrono::milliseconds(DALTA_DUATION) - (exeEnd - exeStart);
			m_curDegree = t_degree;
			this_thread::sleep_for(sleepTime);
		}

		ushort pulseWidth = ConvertDegreeToPulseWidth(degree);
		if (!SetPulseWidth(pulseWidth))
		{
			printf("Fail to set servo motor degree\n");
			return false;
		}

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

		int ret = wiringPiI2CWriteReg16(I2C_FD, m_channel, 0);
		if (ret < 0)
			return -1;

		int first = wiringPiI2CRead(I2C_FD);
		if (first < 0)
			return -1;

		int second = wiringPiI2CRead(I2C_FD);
		if (second < 0)
			return -1;

		return (first << 8) + second;
	}
}
