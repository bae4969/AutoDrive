#include "Modules/PiCar.h"

using namespace std;

int main()
{
    PiCar::PiCar car;
    if (!car.Init())
        return -1;

    car.RemoteRun();
    car.Release();

    return 0;
}
