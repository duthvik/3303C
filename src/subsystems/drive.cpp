#include "main.h"

MotorGroup leftDrive({-1,-2,-3});
MotorGroup rightDrive({4,5,6});

Controller controller;
updateDrive(){
    double leftPower = controller.getAnalog(ControllerAnalog::leftY);
    double rightPower = controller.getAnalog(ControllerAnalog::rightY);
    leftDrive.
}
