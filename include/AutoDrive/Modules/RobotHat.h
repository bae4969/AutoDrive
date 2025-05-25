#pragma once
#include <atomic>

namespace RobotHat
{
	typedef unsigned short ushort;
	enum CAR_I2C_CHANNEL
	{
		CAR_I2C_CHANNEL_CAMERA_YAW = 0,
		CAR_I2C_CHANNEL_CAMERA_TILT = 1,
		CAR_I2C_CHANNEL_FRONT_STEER = 2,
		CAR_I2C_CHANNEL_REAR_LEFT = 13,
		CAR_I2C_CHANNEL_REAR_RIGHT = 12,
	};

	bool InitRobotHat();

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
}
