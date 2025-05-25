#include "EP0152.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <shared_mutex>

namespace EP0152
{
	using namespace std;
	using namespace cv;

	static shared_mutex i2cWriteMutex;
	static int LCD_I2C_FD = -1;
	static const int LCD_I2C_ADDRESS = 0x3C;

	bool i2cWriteByte(int fd, uint8_t control, uint8_t data)
	{
		uint8_t buffer[2] = {control, data};
		return write(fd, buffer, 2) == 2;
	}

	bool InitEP0152()
	{
		LCD_I2C_FD = open("/dev/i2c-1", O_RDWR);
		if (LCD_I2C_FD < 0)
		{
			printf("Failed to open I2C bus");
			return false;
		}

		if (ioctl(LCD_I2C_FD, I2C_SLAVE, LCD_I2C_ADDRESS) < 0)
		{
			printf("Failed to set I2C address");
			close(LCD_I2C_FD);
			LCD_I2C_FD = -1;
			return false;
		}

		i2cWriteByte(LCD_I2C_FD, 0x00, 0xAE); // SSD1306_DISPLAY_OFF

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

		i2cWriteByte(LCD_I2C_FD, 0x00, 0xAE); // SSD1306_DISPLAY_OFF
	}
	bool LCD_I2C::Init()
	{
		if (LCD_I2C_FD < 0)
		{
			printf("LCD I2C FD is not initialized\n");
			return false;
		}

		if (i2cWriteByte(LCD_I2C_FD, 0x00, 0xAE) == false ||		   // SSD1306_DISPLAY_OFF
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xD5) == false ||		   // SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x80) == false ||		   // 0x80
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xA8) == false ||		   // SSD1306_SET_MULTIPLEX_RATIO
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x1F) == false ||		   // 0x1F
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xD3) == false ||		   // SSD1306_SET_DISPLAY_OFFSET
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x00) == false ||		   // 0x00
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x40 | 0x00) == false || // SSD1306_SET_START_LINE
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x8D) == false ||		   // SSD1306_CHARGE_PUMP
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x14) == false ||		   // 0x14
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x20) == false ||		   // SSD1306_MEMORY_ADDR_MODE
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x00) == false ||		   // 0x00
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xA0 | 0x01) == false || // SSD1306_SET_SEGMENT_REMAP
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xC8) == false ||		   // SSD1306_COM_SCAN_DIR_DEC
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xDA) == false ||		   // SSD1306_SET_COM_PINS
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x02) == false ||		   // 0x02
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x81) == false ||		   // SSD1306_SET_CONTRAST_CONTROL
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x8F) == false ||		   // 0x8F
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xD9) == false ||		   // SSD1306_SET_PRECHARGE_PERIOD
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xF1) == false ||		   // 0xF1
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xDB) == false ||		   // SSD1306_SET_VCOM_DESELECT
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x40) == false ||		   // 0x40
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xA4) == false ||		   // SSD1306_DISPLAY_ALL_ON_RESUME
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xA6) == false ||		   // SSD1306_NORMAL_DISPLAY
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x2E) == false ||		   // SSD1306_DEACTIVATE_SCROLL
			i2cWriteByte(LCD_I2C_FD, 0x00, 0xAF) == false)		   		// SSD1306_DISPLAY_ON
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
		if (i2cWriteByte(LCD_I2C_FD, 0x00, 0x21) == false ||	// SSD1306_COLUMNADDR
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x00) == false ||	// Column start address. (0 = reset)
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x7F) == false ||	// Column end address
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x22) == false ||	// SSD1306_PAGEADDR
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x00) == false ||	// Page start address
			i2cWriteByte(LCD_I2C_FD, 0x00, 0x03) == false)		// Page end address
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

				if (i2cWriteByte(LCD_I2C_FD, 0x40, totData) == false)
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
