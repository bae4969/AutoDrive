#include "PiCar.h"
#include "Logger.h"
#include "INIParser.h"
#include <csignal>

using namespace std;

PiCar::PiCar car;

void signalHandler(int signum)
{
    printf("\n");
    printf("---------------------------------\n");
    printf("|    Rised interrupt signal!    |\n");
    printf("---------------------------------\n");
    printf("\n");
    car.Stop();
}

int main(int argc, char **argv)
{
    INIParser iniParser;
    iniParser.Load("./AutoDrive.ini");
    Logger::InitLogger(
        iniParser.GetValue("logging", "file_path", "logs/autodrive.log"),
        iniParser.GetInt("logging", "file_size_kb", 1024),
        iniParser.GetInt("logging", "rotated_files", 1));
    LOG_INFO("AutoDrive starting (argc={})", argc);

    signal(SIGINT, signalHandler);

    try
    {
        PiCar::PICAR_MODE mode;
        if (argc < 2)
            mode = PiCar::PICAR_MODE_DIRECT;
        else
            switch (atoi(argv[1]))
            {
            case 1:
                mode = PiCar::PICAR_MODE_DIRECT;
                break;
            case 2:
                mode = PiCar::PICAR_MODE_REMOTE;
                break;
            case 3:
                mode = PiCar::PICAR_MODE_CAMERA;
                break;
            default:
                throw std::exception();
            }

        if (!car.Init(mode))
            return -2;
    }
    catch (...)
    {
        LOG_ERROR("Argument must be integer value");
        LOG_INFO("1 : Direct control mode");
        LOG_INFO("2 : Remote control mode");
        LOG_INFO("3 : Camera mode");
        return -1;
    }

    try
    {
        car.Run();
    }
    catch (...)
    {
        LOG_EXC_ERROR("Fail to run PiCar");
        return -3;
    }

    try
    {
        car.Release();
    }
    catch (...)
    {
        LOG_EXC_ERROR("Fail to release PiCar");
        return -4;
    }

    LOG_INFO("End of output");
    return 0;
}
