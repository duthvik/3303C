#include "main.h"
#include "okapi/impl/device/controllerUtil.hpp"

Motor sling(11, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

bool stop=true;

void updateSling()
{
    if(controller.getDigital(ControllerDigital::L1))
    {
        sling.moveVoltage(10000);
    }

    else if(controller.getDigital(ControllerDigital::L2)&&stop==false)
    {
        sling.moveAbsolute(100, 90);
        stop=true;
    }

    else if(controller.getDigital(ControllerDigital::A))
    {
        stop=false;
        sling.moveVoltage(0);
    }
    
    else if(stop==true)
    {
        sling.moveVoltage(0);
    }
}