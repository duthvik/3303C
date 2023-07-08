#include "main.h"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"
#include "pros/motors.hpp"

extern Motor sling1;
extern Motor sling2;
extern MotorGroup sling;
void moveSling(double pos);
void updateSling();
extern RotationSensor rotation;