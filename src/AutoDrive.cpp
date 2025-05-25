#include "PiCar.h"
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
        printf("Argument must be integer value\n");
        printf("1 : Direct control mode\n");
        printf("2 : Remote control mode\n");
        printf("3 : Camera mode\n");
        return -1;
    }

    try
    {
        car.Run();
    }
    catch (...)
    {
        printf("Fail to run PiCar\n");
        return -3;
    }

    try
    {
        car.Release();
    }
    catch (...)
    {
        printf("Fail to release PiCar\n");
        return -4;
    }

    printf("End of output\n");
    return 0;
}
