#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

Motor sling(11 /*port num*/ , false /*if motor is reverse set to true*/, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

bool stop=true;

void moveSling(double pos)
{
    PID slingPID = PID(pos-sling.getPosition(), 0.2, 0, 0, 0, 1.5, 300, 1500);
    while(slingPID.is_settled()==false)
    {
        sling.moveVoltage(slingPID.compute(pos-sling.getPosition()));
        pros::delay(5);
    }
    sling.setBrakeMode(AbstractMotor::brakeMode::hold);
    sling.moveVoltage(0);
}





void updateSling(void* param)
{
  while(true)
  {
    if(controller.getDigital(ControllerDigital::A))
    {
        moveSling(300);
    }

    else if(controller.getDigital(ControllerDigital::L1))
    {
        sling.tarePosition();
        moveSling(-20);
        pros::delay(100);
        sling.tarePosition();
    }
    pros::delay(7);
  }
}

/*in pros terminal, type pros mut*/