#include "Camera.h"
#include "Logger.h"
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
			LOG_EXC_ERROR("Fail to convert camera buffer to Mat");
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

		if (m_cameraManager.start() < 0)
		{
			LOG_ERROR("Fail to start camera manager");
			return false;
		}

		if (m_cameraManager.cameras().size() < 2)
		{
			LOG_ERROR("Two camera was not found [{}]", m_cameraManager.cameras().size());
			return false;
		}

		if (startCamera(0) == false)
		{
			LOG_ERROR("Fail to init camera 0");
			return false;
		}
		if (startCamera(1) == false)
		{
			LOG_ERROR("Fail to init camera 1");
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
			LOG_ERROR("Fail to acquire camera {}", cam_idx);
			return false;
		}

		auto &cameraConfig = m_config[cam_idx] = camera->generateConfiguration({libcamera::StreamRole::VideoRecording});
		if (!cameraConfig)
		{
			LOG_ERROR("Fail to generate configuration for camera {}", cam_idx);
			return false;
		}
		cameraConfig->at(0).pixelFormat = libcamera::formats::RGB888;
		cameraConfig->at(0).size.width = static_cast<uint32_t>(m_imageSize.width);
		cameraConfig->at(0).size.height = static_cast<uint32_t>(m_imageSize.height);
		cameraConfig->at(0).bufferCount = 4;
		cameraConfig->validate();
		if (camera->configure(cameraConfig.get()) < 0)
		{
			LOG_ERROR("Fail to configure camera {}", cam_idx);
			return false;
		}

		auto &allocator = m_allocator[cam_idx] = make_unique<libcamera::FrameBufferAllocator>(camera);
		for (libcamera::StreamConfiguration &cfg : *cameraConfig)
		{
			if (allocator->allocate(cfg.stream()) < 0)
			{
				LOG_ERROR("Fail to allocate buffer for camera {}", cam_idx);
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
				LOG_ERROR("Fail to create request for camera {}", cam_idx);
				continue;
			}
			if (request->addBuffer(cameraStream, buf.get()) < 0)
			{
				LOG_ERROR("Fail to add buffer to request for camera {}", cam_idx);
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

					if (m_frameCount[cam_idx] % (m_frameRate * 10) == 0)
					{
						auto elapsed = currentTime - m_frameCounterStart[cam_idx];
						LOG_INFO("Camera {} FPS: {}", cam_idx, m_frameCount[cam_idx] * 1000.0 / chrono::duration_cast<chrono::milliseconds>(elapsed).count());
						m_frameCounterStart[cam_idx] = currentTime;
						m_frameCount[cam_idx] = 0;
					}
				}
				catch (...)
				{
					LOG_EXC_ERROR("Fail to execute 'requestCompleted' callback for camera {}", cam_idx);
				}

				request->reuse(libcamera::Request::ReuseBuffers);
				m_camera[cam_idx]->queueRequest(request);
			});

		int64_t delta_time = 1000000.0 / m_frameRate;
		auto camcontrols = unique_ptr<libcamera::ControlList>(new libcamera::ControlList());
		camcontrols->set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({delta_time, delta_time}));

		if (camera->start(camcontrols.get()) < 0)
		{
			LOG_ERROR("Failed to start camera {}", cam_idx);
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
