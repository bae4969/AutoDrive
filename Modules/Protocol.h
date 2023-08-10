#pragma once
#include <chrono>

namespace Protocol
{
	typedef unsigned short ushort;
	enum GPIO_PIN
	{
		GPIO_PIN_REAR_LEFT_DIRECTION = 4,
		GPIO_PIN_REAR_RIGHT_DIRECTION = 5,
		GPIO_PIN_SONIC_TRING = 2,
		GPIO_PIN_SONIC_ECHO = 3,
		GPIO_PIN_SWITCH = 24,
		GPIO_PIN_LED = 25,
	};
	enum I2C_CHANNEL
	{
		I2C_CHANNEL_CAMERA_YAW = 0,
		I2C_CHANNEL_CAMERA_TILT = 1,
		I2C_CHANNEL_FRONT_STEER = 2,
		I2C_CHANNEL_REAR_LEFT = 13,
		I2C_CHANNEL_REAR_RIGHT = 12,
	};

	bool InitProtocol();

	class GPIO
	{
	private:
		int m_pinIdx = -1;
		bool m_isOut;
		int m_isHigh;

	public:
		bool Init(int pinIdx, bool isOut);
		bool ChangeMode(bool isOut);
		bool SetOutput(bool isHigh);
		int GetOutput();
		int GetInput();
	};

	class I2C
	{
	private:
		const int PULSE_WIDTH_REG_OFFSET = 0x20;
		const int FREQUENCY_REG_OFFSET = 0x30;
		const int PRESCALER_REG_OFFSET = 0x40;
		const int PERIOD_REG_OFFSET = 0x44;
		int m_channel = -1;
		int m_group = -1;
		ushort m_pulseWidth = 0;
		ushort m_frequency = 50;
		ushort m_prescaler = 10;
		ushort m_period = 4095;
		ushort ConvertBig2Little(ushort value);

	public:
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

	class PWMMotor : public I2C
	{
	private:
		ushort m_minValue = 0;
		ushort m_maxValue = 1;
		ushort m_curValue = 0;
		float m_throttle = 0.0f;

	public:
		bool Init(int channel);
		bool SetMinMax(ushort min, ushort max);
		bool SetValue(ushort value);
		bool SetThrottle(float throttle);
		ushort GetMinValue();
		ushort GetMaxValue();
		ushort GetCurrnetValue();
		float GetThrottle();
	};

	class ServoMotor : public I2C
	{
	private:
		const std::chrono::milliseconds DALTA_DUATION = std::chrono::milliseconds(10);
		float m_defaultDegree = 0.0f;
		float m_curDegree = 0.0f;

		ushort ConvertDegreeToPulseWidth(float degree);

	public:
		bool Init(int channel, float defaultDegree, bool isSetZero = true);
		bool SetDegreeWithTime(float degree, int millisecond = 0);
		bool SetDegreeWithSpeed(float degree, float absDegreePerSecond = 0.0f);
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
}
