#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

namespace Camera
{
	DirectCamera::DirectCamera()
	{
		m_threadStop = true;
	}
	DirectCamera::~DirectCamera()
	{
		m_threadStop = true;
		m_captureThread.join();
		m_rasCam.release();
	}
	bool DirectCamera::Init(int w, int h, int bufSize, int frameRate)
	{
		m_threadStop = true;

		m_frameRate = frameRate;
		m_bufferIndex = 0;
		m_bufferSize = bufSize;
		m_imageSize = Size(w, h);
		m_frameBuffer.resize(m_bufferSize);

		m_rasCam.set(CAP_PROP_FORMAT, CV_8UC3);
		m_rasCam.set(CAP_PROP_FRAME_WIDTH, m_imageSize.width);
		m_rasCam.set(CAP_PROP_FRAME_HEIGHT, m_imageSize.height);
		m_rasCam.set(CAP_PROP_FPS, m_frameRate);
		m_rasCam.set(CAP_PROP_EXPOSURE, 8);
		m_rasCam.set(CAP_PROP_GAIN, 100);

		// cv::CAP_PROP_CONTRAST: [0,100]
		// cv::CAP_PROP_SATURATION: [0,100]
		// cv::CAP_PROP_GAIN: (iso): [0,100]
		// cv::CAP_PROP_EXPOSURE: -1 auto. [1,100] shutter speed from 0 to 33ms
		// cv::CAP_PROP_WHITE_BALANCE_RED_V : [1,100] -1 auto whitebalance
		// cv::CAP_PROP_WHITE_BALANCE_BLUE_U : [1,100] -1 auto whitebalance
		// cv::CAP_PROP_MODE : [1,7] 0 auto mode

		for (auto &t_buf : m_frameBuffer)
			t_buf.IsSet = false;

		m_rasCam.release();
		if (!m_rasCam.open())
		{
			cout << "Error opening camera" << endl;
			return false;
		}
		cout << "Wait for stabilizing camera ..." << endl;
		this_thread::sleep_for(chrono::seconds(3));

		m_threadStop = false;
		m_captureThread = thread(&DirectCamera::insertNewFrame, this);

		return true;
	}
	void DirectCamera::insertNewFrame()
	{
		while (!m_threadStop)
		{
			int nextBufIdx = m_bufferIndex + 1;
			if (nextBufIdx >= m_bufferSize)
				nextBufIdx = 0;

			if (!m_rasCam.grab()){
				printf("Fail to grab image\n");
				continue;
			}

			m_bufferMutex.lock();
			m_frameBuffer[nextBufIdx].IsSet = true;
			m_frameBuffer[nextBufIdx].Time = chrono::steady_clock::now();
			m_rasCam.retrieve(m_frameBuffer[nextBufIdx].Image);
			m_bufferIndex = nextBufIdx;
			m_bufferMutex.unlock();
		}
	}
	Size DirectCamera::GetSize()
	{
		return Size(m_rasCam.get(CAP_PROP_FRAME_WIDTH), m_rasCam.get(CAP_PROP_FRAME_HEIGHT));
	}
	bool DirectCamera::GetFrame(ImageInfo &out_imageInfo, int offset)
	{
		if (offset < 0 || offset >= m_bufferSize)
		{
			printf("Invalid offset valud\n");
			return false;
		}

		m_bufferMutex.lock();
		int lastBufferIndex = m_bufferIndex - offset;
		if (lastBufferIndex < 0)
			lastBufferIndex += m_bufferSize;
		out_imageInfo = m_frameBuffer[lastBufferIndex];
		m_bufferMutex.unlock();

		return out_imageInfo.IsSet;
	}
}
