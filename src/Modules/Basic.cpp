#include "Basic.h"
#include <wiringPi.h>

namespace Basic
{
	using namespace std;

	static bool IsSetGPIO = false;

	bool InitBasic()
	{
		IsSetGPIO = wiringPiSetup() == 0;
		if (!IsSetGPIO)
		{
			printf("Fail to init GPIO\n");
			return false;
		}

		return true;
	}

	bool GPIO::Init(int pinIdx, bool isOut)
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not initialized\n");
			return false;
		}

		pinMode(pinIdx, isOut);
		m_pinIdx = pinIdx;
		m_isOut = isOut;

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

		pinMode(m_pinIdx, isOut);
		m_isOut = isOut;

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

		digitalWrite(m_pinIdx, isHigh);
		m_isHigh = isHigh;

		return true;
	}
	int GPIO::GetOutput()
	{
		if (!IsSetGPIO)
		{
			printf("GPIO is not setted\n");
			return false;
		}

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
			return false;
		}

		if (m_isOut)
		{
			printf("This GPIO %d is not in mode\n", m_pinIdx);
			return -1;
		}

		m_isHigh = digitalRead(m_pinIdx);

		return m_isHigh;
	}
}
