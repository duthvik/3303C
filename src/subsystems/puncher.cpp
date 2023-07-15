#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"
#include <iostream>

Motor sling1(10 /*port num*/ , true /*if motor is reverse set to true*/, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor sling2(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
bool stop=true;

MotorGroup sling({sling1, sling2});



RotationSensor rotation(18);

void moveSling(double pos)
{
    // sling1.setBrakeMode(AbstractMotor::brakeMode::hold);
    // sling2.setBrakeMode(AbstractMotor::brakeMode::hold);
    // PID slingPID = PID(pos-rotation.get(), 5, 0.003, 35, 15, 10, 1000, 5000);
    // while(slingPID.is_settled()==false)
    // {   
    //     sling1.moveVoltage(slingPID.compute(pos-rotation.controllerGet())*1000);
    //     sling2.moveVoltage(slingPID.compute(pos-rotation.controllerGet())*1000);
    //     std::cout << slingPID.compute(pos-rotation.controllerGet()*1000) << "     " << rotation.controllerGet()<<"\n";
    //     pros::delay(5);
    // }
    // sling.setBrakeMode(AbstractMotor::brakeMode::hold);
    // sling.moveVoltage(0);



  IterativePosPIDController drivePID = IterativeControllerFactory::posPID(0.095,0.001,0.00);

  // Set the target distance for the drive
  drivePID.setTarget(pos);

  // Get the initial position of the left and right drive motors
  double initLeftPos = sling1.getPosition();
  double initRightPos = sling2.getPosition();
  int timer = 0;

  bool isSettled=false;
  double error=pos-rotation.controllerGet();
  double starti=15;
  double accumError=0;
  double settleError=2;
  double settleTime=300;
  double settlingTime=0;
  double runningTime=0;
  double timeout=1500;

 
  



  // Use a while loop to repeatedly check the current position of the drive motors
  while (isSettled==false) {
    // Calculate the current position of the left and right drive motors
    double currentLeftPos = sling1.getPosition();
    double currentRightPos = sling2.getPosition();
    error=rotation.controllerGet();

    // Use the PID controller to calculate the necessary velocity for the drive motors
    double vel = drivePID.step(rotation.controllerGet());
    std::cout << vel<< "    "<< rotation.controllerGet()<<"\n";

    // Set the drive motors to the calculated velocity
    sling1.moveVoltage(vel*12000);
    sling2.moveVoltage(vel*12000);


     if(fabs(error) < starti) accumError+=error;
  if(fabs(error)<settleError){
    settlingTime+=10;
  } 
  else {
    settlingTime = 0;
  }
  runningTime+=10;

  if (runningTime>timeout && timeout != 0){
    isSettled=true;
  }
  else if (settlingTime>settleTime){
    isSettled =(true);
  }

  else{
    isSettled=false;
  }

    // Delay for a short period of time
    pros::delay(10);

    timer += 10;
  }

  // Stop the drive motors
  sling1.moveVoltage(0);
  sling2.moveVoltage(0);
 }








ControllerButton go(ControllerDigital::A);

void updateSling()
{
  while(true)
  //{
    //
    //if(go.changedToPressed())
    //{
   //     sling1.tarePosition();
   //     sling2.tarePosition();
   //     pros::delay(200);
   //     moveSling(0);
  //  }

  //   else if(controller.getDigital(ControllerDigital::L1))
  //   {
  //       sling.tarePosition();
  //       moveSling(-20);
  //       pros::delay(100);
  //       sling.tarePosition();
  //   }
  //   pros::delay(7);
  // }

  if(controller.getDigital(ControllerDigital::L1))
  {
    sling1.moveVoltage(-12000);
    sling2.moveVoltage(-12000);
  }

  else if(controller.getDigital(ControllerDigital::L2))
  {
    sling1.moveVoltage(12000);
    sling2.moveVoltage(12000);
  }
  else{
    sling1.moveVoltage(0);
    sling2.moveVoltage(0);
  } 
  pros::delay(5);
}
}

/*in pros terminal, type pros mut*/