#include "main.h"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/adi.hpp"

pros::ADIDigitalOut claw('A');
pros::ADIDigitalOut clawIn('B');

void updateClaw()
{
    while(true){
        if(controller.getDigital(ControllerDigital::R1))
    {
        claw.set_value(true);
        pros::delay(10);
        clawIn.set_value(true);
    }

    if(controller.getDigital(ControllerDigital::R2))
    {
        claw.set_value(false);
        pros::delay(500);
        clawIn.set_value(false);
    }
        pros::delay(10);
    }
    
}