#pragma once
#include <raspicam/raspicam_cv.h>
#include <opencv2/core.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

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
        bool m_threadStop;
        std::mutex m_bufferMutex;
        std::thread m_captureThread;

        raspicam::RaspiCam_Cv m_rasCam;
        int m_frameRate;
        int m_bufferIndex;
        int m_bufferSize;
        cv::Size m_imageSize;
        std::vector<ImageInfo> m_frameBuffer;

        void insertNewFrame();

    public:
        DirectCamera();
        ~DirectCamera();
        bool Init(int w = 1280, int h = 960, int bufSize = 120, int frameRate = 30);
        cv::Size GetSize();
        bool GetFrame(ImageInfo& out_imageInfo, int offset = 0);
    };
}
