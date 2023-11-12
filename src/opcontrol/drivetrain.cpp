#include "main.h"

void DrivetrainLoop(void *_)
{
    while (true)
    {
        thing->arcade(controller.getAnalog(ControllerAnalog::rightX), controller.getAnalog(ControllerAnalog::leftY));

        pros::delay(20);
    }
}