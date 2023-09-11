#include "EP0152.h"
#include <wiringPiI2C.h>

namespace EP0152
{
	using namespace std;
	using namespace cv;

	static mutex i2cWriteMutex;
	static int LCD_I2C_FD = -1;
	static const int LCD_I2C_ADDRESS = 0x3C;

	bool InitEP0152()
	{
		LCD_I2C_FD = wiringPiI2CSetupInterface("/dev/i2c-1", LCD_I2C_ADDRESS);
		if (LCD_I2C_FD < 0)
		{
			printf("Fail to init LCD I2C FD\n");
			return false;
		}

		return true;
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
		if (LCD_I2C_FD < 0)
		{
			printf("LCD I2C FD is not initialized\n");
			return false;
		}

		if (wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xAE) < 0 ||		   // SSD1306_DISPLAY_OFF
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xD5) < 0 ||		   // SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x80) < 0 ||		   // 0x80
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xA8) < 0 ||		   // SSD1306_SET_MULTIPLEX_RATIO
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x1F) < 0 ||		   // 0x1F
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0xD3) < 0 ||		   // SSD1306_SET_DISPLAY_OFFSET
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x00) < 0 ||		   // 0x00
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x40 | 0x00) < 0 || // SSD1306_SET_START_LINE
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
		if (wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x21) < 0 || // SSD1306_COLUMNADDR
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x00) < 0 || // Column start address. (0 = reset)
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x7F) < 0 || // Column end address
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x22) < 0 || // SSD1306_PAGEADDR
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x00) < 0 || // Page start address
			wiringPiI2CWriteReg8(LCD_I2C_FD, 0x00, 0x03) < 0)	// Page end address
		{
			printf("Fail to reset LCD I2C\n");
			return false;
		}

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
}
