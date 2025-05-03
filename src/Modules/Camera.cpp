#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <shared_mutex>
#include <sys/mman.h>
#include <fcntl.h>

namespace Camera
{
	using namespace std;
	using namespace cv;

	bool ConvertBufferToMat(Mat &out_mat, libcamera::FrameBuffer *buffer)
	{
		try
		{
			const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
			int fd = plane.fd.get();
			size_t length = plane.length;

			void *mem = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (mem == MAP_FAILED)
				throw std::runtime_error("mmap failed");

			memcpy(out_mat.data, mem, out_mat.total() * out_mat.elemSize());
			munmap(mem, length);

			return true;
		}
		catch (...)
		{
			return false;
		}
	}

	DirectCamera::DirectCamera()
	{
	}
	DirectCamera::~DirectCamera()
	{
		m_camera[0]->stop();
		m_camera[1]->stop();
		m_cameraManager.stop();
	}
	bool DirectCamera::Init(int w, int h, int bufSize, int frameRate)
	{
		m_frameRate = frameRate;
		m_bufferSize = bufSize;
		m_imageSize = Size(w, h);
		m_bufferIndex[0] = 0;
		m_bufferIndex[1] = 0;
		m_frameBuffer[0].resize(m_bufferSize);
		m_frameBuffer[1].resize(m_bufferSize);
		for (auto &t_list : m_frameBuffer)
		{
			for (auto &t_buf : t_list)
			{
				t_buf.IsSet = false;
				t_buf.Time = chrono::steady_clock::now();
				t_buf.Image = Mat::zeros(m_imageSize, CV_8UC3);
			}
		}

		m_cameraManager.start();
		if (m_cameraManager.cameras().size() < 2)
		{
			printf("Two camera was not found\n");
			return false;
		}

		if (startCamera(0) == false)
		{
			printf("Fail to init camera 0\n");
			return false;
		}
		if (startCamera(1) == false)
		{
			printf("Fail to init camera 1\n");
			return false;
		}

		for (auto &request : m_requests[0])
			m_camera[0]->queueRequest(request.get());
		for (auto &request : m_requests[1])
			m_camera[1]->queueRequest(request.get());

		return true;
	}
	bool DirectCamera::startCamera(int cam_idx)
	{
		const auto &camera = m_camera[cam_idx] = m_cameraManager.cameras()[cam_idx];
		if (camera->acquire() < 0)
		{
			printf("Fail to acquire camera %d\n", cam_idx);
			return false;
		}

		auto &cameraConfig = m_config[cam_idx] = camera->generateConfiguration({libcamera::StreamRole::VideoRecording});
		if (!cameraConfig)
		{
			printf("Fail to generate configuration for camera %d\n", cam_idx);
			return false;
		}
		cameraConfig->at(0).pixelFormat = libcamera::formats::RGB888;
		cameraConfig->at(0).size.width = static_cast<uint32_t>(m_imageSize.width);
		cameraConfig->at(0).size.height = static_cast<uint32_t>(m_imageSize.height);
		cameraConfig->at(0).bufferCount = 4;
		cameraConfig->validate();
		if (camera->configure(cameraConfig.get()) < 0)
		{
			printf("Fail to configure camera %d\n", cam_idx);
			return false;
		}

		auto &allocator = m_allocator[cam_idx] = make_unique<libcamera::FrameBufferAllocator>(camera);
		for (libcamera::StreamConfiguration &cfg : *cameraConfig)
		{
			if (allocator->allocate(cfg.stream()) < 0)
			{
				printf("Fail to allocate buffer for camera %d\n", cam_idx);
				return false;
			}
		}
		auto &cameraStream = m_stream[cam_idx] = cameraConfig->at(0).stream();
		const auto &buffers = allocator->buffers(cameraStream);
		for (const auto &buf : buffers)
		{
			auto request = camera->createRequest();
			if (!request)
			{
				printf("Fail to create request for camera %d\n", cam_idx);
				continue;
			}
			if (request->addBuffer(cameraStream, buf.get()) < 0)
			{
				printf("Fail to add buffer to request for camera %d\n", cam_idx);
				continue;
			}
			m_requests[cam_idx].push_back(std::move(request));
		}

		camera->requestCompleted.connect(
			this,
			[this, cam_idx](libcamera::Request *request)
			{
				try
				{
					if (request->status() != libcamera::Request::RequestComplete)
						throw std::runtime_error("Request not complete");

					auto currentTime = chrono::steady_clock::now();
					{
						auto *buffer = request->buffers().at(m_stream[cam_idx]);
						unique_lock lock(m_bufferMutex[cam_idx]);
						int nextBufIdx = (m_bufferIndex[cam_idx] + 1) % m_bufferSize;
						auto &imageInfo = m_frameBuffer[cam_idx][nextBufIdx];
						imageInfo.IsSet = ConvertBufferToMat(imageInfo.Image, buffer);
						if (imageInfo.IsSet)
						{
							imageInfo.Time = currentTime;
							m_bufferIndex[cam_idx] = nextBufIdx;
							m_frameCount[cam_idx]++;
						}
					}

					if (m_frameCount[cam_idx] % m_frameRate == 0)
					{
						auto elapsed = currentTime - m_frameCounterStart[cam_idx];
						printf("Camera %d FPS: %f\n", cam_idx, m_frameCount[cam_idx] * 1000.0 / chrono::duration_cast<chrono::milliseconds>(elapsed).count());
						m_frameCounterStart[cam_idx] = currentTime;
						m_frameCount[cam_idx] = 0;
					}
				}
				catch (...)
				{
					printf("Fail to execute 'requestCompleted' callback for camera %d\n", cam_idx);
				}

				request->reuse(libcamera::Request::ReuseBuffers);
				m_camera[cam_idx]->queueRequest(request);
			});

		int64_t delta_time = 1000000.0 / m_frameRate;
		auto camcontrols = unique_ptr<libcamera::ControlList>(new libcamera::ControlList());
		camcontrols->set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({delta_time, delta_time}));
			
		if (camera->start(camcontrols.get()) < 0)
		{
			printf("Failed to start camera %d\n", cam_idx);
			return false;
		}

		return true;
	}

	Size DirectCamera::GetSize()
	{
		return m_imageSize;
	}
	int DirectCamera::GetFrameRate()
	{
		return m_frameRate;
	}
	bool DirectCamera::GetFrame(ImageInfo &out_leftImageInfo, ImageInfo &out_rightImageInfo)
	{
		{
			unique_lock lock(m_bufferMutex[0]);
			out_leftImageInfo = m_frameBuffer[0][m_bufferIndex[0]];
		}

		{
			unique_lock lock(m_bufferMutex[1]);
			out_rightImageInfo = m_frameBuffer[1][m_bufferIndex[1]];
		}

		return out_leftImageInfo.IsSet && out_rightImageInfo.IsSet;
	}
}
