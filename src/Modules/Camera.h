#pragma once
#include <boost/shared_ptr.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <opencv2/core.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <shared_mutex>

namespace Camera
{
    typedef std::chrono::steady_clock::time_point TimeType;

    struct ImageInfo{
        bool IsSet;
        TimeType Time;
        cv::Mat Image;
    };

    class DirectCamera
    {
    private:
        libcamera::CameraManager m_cameraManager;
        std::shared_ptr<libcamera::Camera> m_camera[2];
        std::unique_ptr<libcamera::FrameBufferAllocator> m_allocator[2];
        std::unique_ptr<libcamera::CameraConfiguration> m_config[2];
        libcamera::Stream* m_stream[2];
		std::vector<std::unique_ptr<libcamera::Request>> m_requests[2];
        std::chrono::steady_clock::time_point m_frameCounterStart[2];
        int m_frameCount[2];
        
        int m_frameRate;
        int m_bufferSize;
        cv::Size m_imageSize;

        int m_bufferIndex[2];
        std::shared_mutex m_bufferMutex[2];
        std::vector<ImageInfo> m_frameBuffer[2];

        bool startCamera(int cam_idx);

    public:
        DirectCamera();
        ~DirectCamera();
        bool Init(int w = 1280, int h = 960, int bufSize = 120, int frameRate = 30);
        cv::Size GetSize();
        int GetFrameRate();
        bool GetFrame(ImageInfo &out_leftImageInfo, ImageInfo &out_rightImageInfo);
    };
}
