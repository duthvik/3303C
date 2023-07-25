#include "main.h"
#include "okapi/impl/device/button/adiButton.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

ADIButton autonButton('D');

void selector()
{
    int currentAuton = 0;
    bool autonStart = false;

     while(autonStart == false)
     {            //Changing the names below will only change their names on the
        pros::screen::erase();
        switch(currentAuton){       //Tap the brain screen to cycle through autons.
        case 0:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 1:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 2:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 3:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 4:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 5:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 6:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 7:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        }
        if(autonButton.isPressed()){
            currentAuton ++;
        } else if (currentAuton == 8){
            currentAuton = 0;
        }
        pros::delay(10);
      }
}