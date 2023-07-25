#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "pros/rtos.hpp"


Drive chassis(3.25, 0.6,360,3.25,5.2,0,0);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	sling1.tarePosition();
	sling2.tarePosition();
	sling1.setBrakeMode(AbstractMotor::brakeMode::hold);
	sling2.setBrakeMode(AbstractMotor::brakeMode::hold);
	rotation.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
 int currentAuton = 0;
    bool autonStart = false;
void competition_initialize() {


	

     while(autonStart == false)
     {            //Changing the names below will only change their names on the
        pros::screen::erase();
        switch(currentAuton){       //Tap the brain screen to cycle through autons.
        case 0:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 1");
            break;
        case 1:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 2");
            break;
        case 2:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 3");
            break;
        case 3:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 4");
            break;
        case 4:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 5");
            break;
        case 5:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 6");
            break;
        case 6:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 7");
            break;
        case 7:
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Auton 8");
            break;
        }
        if(autonButton.isPressed()){
			while (autonButton.isPressed()) {}
            currentAuton ++;
        } else if (currentAuton == 8){
            currentAuton = 0;
        }
        pros::delay(10);
      }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	 autonStart = true;
  switch(currentAuton){  
    case 0:

      break;        
    case 1:  

      break;
    case 2:
      
      break;
    case 3:
      
      break;
    case 4:
      
      break;
    case 5:
      
      break;
    case 6:
      
      break;
    case 7:
      
      break;
  }
	pros::Task(chassis.print_odom_task);
	pros::Task(chassis.position_track_task);
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
	pros::Task(chassis.print_odom_task);
	pros::Task(chassis.position_track_task);
	pros::Task o(updateSling);
	pros::Task p(updateClaw);
	 while(true)
	 {
	 	 updateDrive();
	 	pros::delay(7);
	 }
	
}
