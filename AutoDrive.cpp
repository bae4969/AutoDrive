#include "Modules/PiCar.h"

using namespace std;

int main(int argc, char **argv)
{
    PiCar::PiCar car;
    try
    {
        int type = atoi(argv[1]);
        PiCar::PICAR_MODE mode;
        if (argc < 2)
            mode = PiCar::PICAR_MODE_DIRECT;
        else
            switch (type)
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

    car.Run();
    car.Release();

    return 0;
}
