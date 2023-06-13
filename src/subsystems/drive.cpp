#include "main.h"

MotorGroup leftDrive({-1,-2,-3});
MotorGroup rightDrive({4,5,6});

Controller controller;
void updateDrive(){
    double leftPower = controller.getAnalog(ControllerAnalog::leftY);
    double rightPower = controller.getAnalog(ControllerAnalog::rightY);
    leftDrive.moveVoltage(leftPower);
    rightDrive.moveVoltage(rightPower);
}
dumb nigger