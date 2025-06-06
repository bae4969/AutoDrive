#include "RobotHat.h"
#include "Basic.h"
#include <wiringPiI2C.h>
#include <vector>
#include <math.h>
#include <chrono>
#include <mutex>
#include <shared_mutex>

namespace RobotHat
{
	using namespace std;

	static shared_mutex i2cWriteMutex;
	static int CAR_I2C_FD = -1;
	static const int CAR_I2C_ADDRESS = 0x14;

	bool InitRobotHat()
	{
		Basic::GPIO rstMCU;
		rstMCU.Init(Basic::GPIO_PIN_MCU_RESET, true);
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
			CAR_I2C_FD = -1;
			return false;
		}

		return true;
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
			printf("CAR I2C FD is not initialized\n");
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
		int ret = -1;
		{
			unique_lock lock(i2cWriteMutex);
			ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		}
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
		int ret = -1;
		{
			unique_lock lock(i2cWriteMutex);
			ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		}
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
		int ret = -1;
		{
			unique_lock lock(i2cWriteMutex);
			ret = wiringPiI2CWriteReg16(CAR_I2C_FD, reg, convertBig2Little(value));
		}
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
			printf("CAR I2C FD is not initialized\n");
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

		int result = -1;
		int first = -1;
		int second = -1;
		{
			shared_lock lock(i2cWriteMutex);
			result = wiringPiI2CWriteReg16(CAR_I2C_FD, m_channel, 0);
			first = wiringPiI2CRead(CAR_I2C_FD);
			second = wiringPiI2CRead(CAR_I2C_FD);
		}

		return result < 0 ? -1 : (first << 8) + second;
	}
}
