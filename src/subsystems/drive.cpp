#include "main.h"

Motor rightTop(18, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMid(1, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(7, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftTop(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMid(5, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(2, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<OdomChassisController> drive =
     ChassisControllerBuilder()
    .withMotors({leftTop, leftMid, leftBack}, {rightTop, rightMid, rightBack})
    .withDimensions(AbstractMotor::gearset::blue, {{4_in, 13.7_in}, okapi::imev5GreenTPR}) //change dimensions
    .withOdometry({{2.75_in, 7_in}, quadEncoderTPR}) //specifies the tracking wheels dimentions (change later)
    .buildOdometry();

MotorGroup leftDrive({-1,-2,-3});
MotorGroup rightDrive({4,5,6});

Controller controller;
void updateDrive(){
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}
dumb nigger