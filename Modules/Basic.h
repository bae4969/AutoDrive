#pragma once
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <chrono>
#include <atomic>
#include <thread>
#include <boost/shared_ptr.hpp>

namespace Basic
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
		GPIO_PIN_LCD_LED_BACK_LEFT = 21, // duplicate pin num with reset
		GPIO_PIN_LCD_LED_BACK_RIGHT = 22,
	};

	bool InitBasic();

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
}
