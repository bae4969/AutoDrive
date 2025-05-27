#include "Basic.h"
#include <gpiod.h>

namespace Basic
{
	using namespace std;

	bool InitBasic()
	{
		return true;
	}

	GPIO::~GPIO()
	{
		if (m_line)
			gpiod_line_release(m_line);
		if (m_chip)
			gpiod_chip_close(m_chip);
		m_line = nullptr;
		m_chip = nullptr;
	}
	bool GPIO::Init(int pinIdx, bool isOut)
	{
		m_pinIdx = pinIdx;
		m_isOut = isOut;

		m_chip = gpiod_chip_open_by_name("gpiochip0");
		if (!m_chip)
		{
			printf("Failed to open gpiochip0");
			return false;
		}

		m_line = gpiod_chip_get_line(m_chip, m_pinIdx);
		if (!m_line)
		{
			printf("Failed to get GPIO line");
			return false;
		}

		struct gpiod_line_request_config config = {
			.consumer = "RobotHat",
			.request_type = m_isOut ? GPIOD_LINE_REQUEST_DIRECTION_OUTPUT : GPIOD_LINE_REQUEST_DIRECTION_INPUT,
			.flags = 0};
		int ret = gpiod_line_request(m_line, &config, 0);
		if (ret < 0)
		{
			printf("Failed to request GPIO line");
			return false;
		}

		return true;
	}
	bool GPIO::ChangeMode(bool isOut)
	{
		if (m_pinIdx < 0)
		{
			printf("This GPIO %d is not init\n", m_pinIdx);
			return false;
		}

		struct gpiod_line_request_config config = {
			.consumer = "RobotHat",
			.request_type = isOut ? GPIOD_LINE_REQUEST_DIRECTION_OUTPUT : GPIOD_LINE_REQUEST_DIRECTION_INPUT,
			.flags = 0};
		int ret = gpiod_line_request(m_line, &config, 0);
		if (ret < 0)
		{
			printf("Failed to request GPIO line");
			return false;
		}

		m_isOut = isOut;

		return true;
	}
	bool GPIO::SetOutput(bool isHigh)
	{
		if (!m_isOut)
		{
			printf("This GPIO %d is not out mode\n", m_pinIdx);
			return false;
		}

		int val = isHigh ? 1 : 0;
		int ret = gpiod_line_set_value(m_line, val);
		if (ret < 0)
		{
			printf("Failed to set GPIO %d value", m_pinIdx);
			return false;
		}

		m_isHigh = val;

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
		if (m_isOut)
		{
			printf("This GPIO %d is not in mode\n", m_pinIdx);
			return -1;
		}

		int value = gpiod_line_get_value(m_line);
		if (value < 0)
		{
			perror("Failed to read GPIO value");
			return -1;
		}

		m_isHigh = value;

		return m_isHigh;
	}
}
