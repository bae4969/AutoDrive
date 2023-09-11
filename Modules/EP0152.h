#pragma once
#include <opencv2/core.hpp>

namespace EP0152
{
	bool InitEP0152();

	class LCD_I2C
	{
		bool m_isInit;
		int m_width;
		int m_height;
		int m_page;
		int m_line;
		cv::Mat m_img8;

	public:
		LCD_I2C();
		~LCD_I2C();

		bool Init();
		bool Reset();
		bool SetImage(cv::Mat img8);
		cv::Mat GetImage();
		cv::Size GetImageSize();
	};
}
