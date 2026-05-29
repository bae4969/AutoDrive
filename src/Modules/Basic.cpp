#include "Basic.h"
#include "Logger.h"
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
			LOG_ERROR("Fail to init GPIO");
			return false;
		}

		return true;
	}

	bool GPIO::Init(int pinIdx, bool isOut)
	{
		if (!IsSetGPIO)
		{
			LOG_ERROR("GPIO is not initialized");
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
			LOG_ERROR("GPIO is not setted");
			return false;
		}

		if (m_pinIdx < 0)
		{
			LOG_ERROR("This GPIO {} is not init", m_pinIdx);
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
			LOG_ERROR("GPIO is not setted");
			return false;
		}

		if (!m_isOut)
		{
			LOG_ERROR("This GPIO {} is not out mode", m_pinIdx);
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
			LOG_ERROR("GPIO is not setted");
			return false;
		}

		if (!m_isOut)
		{
			LOG_ERROR("This GPIO {} is not out mode", m_pinIdx);
			return -1;
		}

		return m_isHigh;
	}
	int GPIO::GetInput()
	{
		if (!IsSetGPIO)
		{
			LOG_ERROR("GPIO is not setted");
			return false;
		}

		if (m_isOut)
		{
			LOG_ERROR("This GPIO {} is not in mode", m_pinIdx);
			return -1;
		}

		m_isHigh = digitalRead(m_pinIdx);

		return m_isHigh;
	}
}
