#include "Modules/PiCar.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

int main()
{
    // PiCar::PiCar car;
    // if (!car.Init())
    //     return -1;

    // car.Run();

    Hardware::CameraSensor camera;
    camera.Init();

    auto start = chrono::system_clock::now();
    auto end = chrono::system_clock::now();
    int frameCount = 0;
    
    while (true)
    {
        this_thread::sleep_for(chrono::milliseconds(60));
        if((end - start).count() > 60000000.0f){
            start = end;
            frameCount = 0;
        }
    }

    return 0;
}
