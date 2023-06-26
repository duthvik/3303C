#include "main.h"
#include "okapi/impl/device/motor/motor.hpp"

extern Motor sling;

void moveSling(double pos);
void updateSling(void* param);