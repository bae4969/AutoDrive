#include "Modules/Camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>

using namespace std;
using namespace cv;

int main()
{
    Camera::DirectCamera camera(1280, 960, 120, 10);
    camera.Init();

    const size_t averageCount = 20;
    const double sensitive = 4.0;
    Camera::ImageInfo imageInfo_old;
    Camera::ImageInfo imageInfo_cur;
    queue<pair<double, double>> valueHistory;
    cout << "Start to watching ...\n";
    while (true)
    {
        try
        {
            bool good_old = camera.GetFrame(imageInfo_old, 5);
            bool good_cur = camera.GetFrame(imageInfo_cur);
            if (!good_old || !good_cur)
                throw exception();

            Mat &img_old = imageInfo_old.Image;
            Mat &img_cur = imageInfo_cur.Image;

            Mat diffOldCur = abs(img_old - img_cur);
            Scalar diffMean, diffStdDev;
            meanStdDev(diffOldCur, diffMean, diffStdDev);

            double diffMeanSum = diffMean[0] + diffMean[1] + diffMean[2];
            double diffStdDevSum = diffStdDev[0] + diffStdDev[1] + diffStdDev[2];
            valueHistory.push(make_pair(diffMeanSum, diffStdDevSum));
            if (valueHistory.size() < averageCount)
                throw exception();

            auto copy = valueHistory;
            valueHistory.pop();

            int count = 0;
            double meanSum = 0.0;
            double stdDevSum = 0.0;
            while (copy.size() > 1)
            {
                auto t_value = copy.front();
                meanSum += t_value.first;
                stdDevSum += t_value.second;
                count++;
                copy.pop();
            }

            double meanAvg = meanSum / count;
            double stdDevAvg = stdDevSum / count;
            double meanThreshold = meanAvg * sensitive;
            double stdDevThreshold = stdDevAvg * sensitive;
            double meanCurrent = copy.front().first;
            double stdDevCurrent = copy.front().second;
            if (meanCurrent < meanThreshold &&
                stdDevCurrent < stdDevThreshold)
                throw exception();

            time_t t = time(nullptr);
            tm *now = localtime(&t);
            char imgFileNameBuf[128]{};
            sprintf(
                imgFileNameBuf,
                "./Output/CCTV/%04d%02d%02d_%02d%02d%02d.png",
                now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
                now->tm_hour, now->tm_min, now->tm_sec);

            imwrite(imgFileNameBuf, img_cur);
        }
        catch (...)
        {
        }

        this_thread::sleep_for(chrono::milliseconds(200));
    }

    return 0;
}
