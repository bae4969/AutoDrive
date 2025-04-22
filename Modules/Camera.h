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
        cv::Mat ImageLeft;
        cv::Mat ImageRight;
    };

    class DirectCamera
    {
    private:
        bool m_threadStop;
        std::shared_mutex m_bufferMutex;
        std::thread m_captureThread;

        libcamera::CameraManager m_cameraManager;
        std::shared_ptr<libcamera::Camera> m_camera0;
        std::shared_ptr<libcamera::Camera> m_camera1;
        // libcamera::CameraConfiguration* m_cameraConfig0;
        // libcamera::CameraConfiguration* m_cameraConfig1;
        // libcamera::FrameBufferAllocator m_allocator0;
        // libcamera::FrameBufferAllocator m_allocator1;
        // libcamera::Request *m_request0;
        // libcamera::Request *m_request1;
        // libcamera::Stream *m_stream0;
        // libcamera::Stream *m_stream1;
        
        int m_frameRate;
        int m_bufferIndex;
        int m_bufferSize;
        cv::Size m_imageSize;
        std::vector<ImageInfo> m_frameBuffer;

        void insertNewFrame();

    public:
        DirectCamera();
        ~DirectCamera();
        bool Init(int w = 640, int h = 480, int bufSize = 120, int frameRate = 30);
        cv::Size GetSize();
        bool GetFrame(ImageInfo& out_imageInfo, int offset = 0);
    };
}
