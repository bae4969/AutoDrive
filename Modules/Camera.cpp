#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

namespace Camera
{
	DirectCamera::DirectCamera(int w, int h, int bufSize, int frameRate)
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
		for (auto &t_buf : m_frameBuffer)
			t_buf.IsSet = false;
	}
	DirectCamera::~DirectCamera()
	{
		m_threadStop = true;
	}
	bool DirectCamera::Init()
	{
		if (!m_rasCam.open())
		{
			cout << "Error opening camera" << endl;
			return false;
		}
		cout << "Wait for stabilizing camera ..." << endl;
		this_thread::sleep_for(chrono::seconds(3));

		m_threadStop = false;
		m_captureThread = thread(&DirectCamera::InsertNewFrame, this);

		return true;
	}
	void DirectCamera::InsertNewFrame()
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
			m_frameBuffer[nextBufIdx].Time = chrono::system_clock::now();
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
