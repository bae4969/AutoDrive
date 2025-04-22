#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <shared_mutex>

namespace Camera
{
	using namespace std;
	using namespace cv;

	DirectCamera::DirectCamera()
	{
		m_threadStop = true;
	}
	DirectCamera::~DirectCamera()
	{
		m_threadStop = true;
		m_captureThread.join();
		m_cameraManager.stop();
	}
	bool DirectCamera::Init(int w, int h, int bufSize, int frameRate)
	{
		m_threadStop = true;

		m_frameRate = frameRate;
		m_bufferIndex = 0;
		m_bufferSize = bufSize;
		m_imageSize = Size(w, h);
		m_frameBuffer.resize(m_bufferSize);
		for (auto &t_buf : m_frameBuffer)
		{
			t_buf.IsSet = false;
			t_buf.Time = chrono::steady_clock::now();
			t_buf.ImageLeft = Mat::zeros(m_imageSize, CV_8UC3);
			t_buf.ImageRight = Mat::zeros(m_imageSize, CV_8UC3);
		}

		m_cameraManager.start();
		const auto &cameras = m_cameraManager.cameras();
		if (cameras.size() < 1)
		{
			cout << "Two camera was not found" << endl;
			return false;
		}

		m_camera0 = cameras[0];
		m_camera1 = cameras[1];
		// 사용권 요청
		if (m_camera0->acquire() ||
			m_camera1->acquire())
		{
			cout << "Fail to acquire camera" << endl;
			return false;
		}

		// m_cameraConfig0 = m_camera0->generateConfiguration({StreamRole::Viewfinder});
		// m_cameraConfig1 = m_camera1->generateConfiguration({StreamRole::Viewfinder});
		// if (!m_cameraConfig0 ||
		// 	!m_cameraConfig1)
		// {
		// 	cout << "Fail to generate camera configuration" << endl;
		// 	return false;
		// }

		// m_cameraConfig0->at(0).pixelFormat = m_cameraConfig1->at(0).pixelFormat = formats::BGR888;
		// m_cameraConfig0->at(0).size = m_cameraConfig1->at(0).size = {m_imageSize.width, m_imageSize.height};
		// m_cameraConfig0->validate();
		// m_cameraConfig1->validate();
		// if (m_camera0->configure(m_cameraConfig0.get()) < 0 ||
		// 	m_camera1->configure(m_cameraConfig1.get()) < 0)
		// {
		// 	cout << "Fail to configure camera" << endl;
		// 	return false;
		// }

		// m_allocator0 = libcamera::FrameBufferAllocator(m_camera0);
		// m_allocator1 = libcamera::FrameBufferAllocator(m_camera1);
		// for (libcamera::StreamConfiguration &cfg : *m_cameraConfig0)
		// {
		// 	if (allocator.allocate(cfg.stream()) < 0)
		// 	{
		// 		cerr << "Failed to allocate buffers" << endl;
		// 		return false;
		// 	}
		// }
		// for (libcamera::StreamConfiguration &cfg : *m_cameraConfig1)
		// {
		// 	if (allocator.allocate(cfg.stream()) < 0)
		// 	{
		// 		cerr << "Failed to allocate buffers" << endl;
		// 		return false;
		// 	}
		// }

		// m_request0 = m_camera0->createRequest();
		// m_request1 = m_camera1->createRequest();
		// if (!m_request0 ||
		// 	!m_request1)
		// {
		// 	cout << "Fail to create request" << endl;
		// 	return false;
		// }

		// m_stream0 = m_cameraConfig0->at(0).stream();
		// m_stream1 = m_cameraConfig1->at(0).stream();
		// const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator.buffers(m_stream0);

		// request->addBuffer(m_stream0, buffers[0].get());

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

			// TODO

			// if (!m_rasCam.grab())
			// {
			// 	printf("Fail to grab image\n");
			// 	continue;
			// }

			unique_lock lock(m_bufferMutex);
			m_frameBuffer[nextBufIdx].IsSet = true;
			m_frameBuffer[nextBufIdx].Time = chrono::steady_clock::now();
			// m_rasCam.retrieve(m_frameBuffer[nextBufIdx].Image);
			m_bufferIndex = nextBufIdx;
		}
	}
	Size DirectCamera::GetSize()
	{
		return m_imageSize;
	}
	bool DirectCamera::GetFrame(ImageInfo &out_imageInfo, int offset)
	{
		if (offset < 0 || offset >= m_bufferSize)
		{
			printf("Invalid offset valud\n");
			return false;
		}

		shared_lock lock(m_bufferMutex);
		int lastBufferIndex = m_bufferIndex - offset;
		if (lastBufferIndex < 0)
			lastBufferIndex += m_bufferSize;
		out_imageInfo = m_frameBuffer[lastBufferIndex];

		return out_imageInfo.IsSet;
	}
}
