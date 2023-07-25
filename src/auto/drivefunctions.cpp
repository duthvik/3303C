#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "pros/rtos.hpp"

pros::Imu inertial(11);

Drive::Drive(double wheel_diameter, double wheel_ratio, double gyro_scale,  double ForwardTracker_diameter, double ForwardTracker_center_distance, double SidewaysTracker_diameter, double SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0)
  //Every device below is passed by port rather than passing the object.
  //This ensures that the devices work properly.

{
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
}

void Drive::drive_with_voltage(double leftVoltage, double rightVoltage){
    leftVoltage = leftVoltage*1000;
    rightVoltage = rightVoltage*1000;
    leftBack.moveVoltage(leftVoltage);
    leftMid.moveVoltage(leftVoltage);
    leftTop.moveVoltage(leftVoltage);
    rightBack.moveVoltage(rightVoltage);
    rightMid.moveVoltage(rightVoltage);
    rightTop.moveVoltage(rightVoltage);
}

// All PID constants are passed as kP, kI, kD, and startI. The kP, kI, and kD are pretty standard,
// but startI keeps the integral value at 0 until the absolute value of the error is below startI.
// This prevents integral windup on bigger turns.

void Drive::set_turn_constants(double turn_max_voltage, double turn_kp, double turn_ki, double turn_kd, double turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(double drive_max_voltage, double drive_kp, double drive_ki, double drive_kd, double drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(double heading_max_voltage, double heading_kp, double heading_ki, double heading_kd, double heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(double swing_max_voltage, double swing_kp, double swing_ki, double swing_kd, double swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

// Settle error and settle time work together to check whether the desired position was achieved, but
// timeout is separate. The robot must stay within the settle error for the duration of the settle time 
// to be settled. If the duration of the movement reaches timeout without being settled, the robot
// gives up and goes to the next movement. 

void Drive::set_turn_exit_conditions(double turn_settle_error, double turn_settle_time, double turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(double drive_settle_error, double drive_settle_time, double drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(double swing_settle_error, double swing_settle_time, double swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

double Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( inertial.get_heading()*360.0/gyro_scale ) ); 
}

double Drive::get_left_position_in(){
  return( leftMid.getPosition()*drive_in_to_deg_ratio );
}

double Drive::get_right_position_in(){
  return( rightMid.getPosition()*drive_in_to_deg_ratio );
}

void Drive::turn_to_angle(double angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(double angle, double turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(double angle, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(double angle, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout, double turn_kp, double turn_ki, double turn_kd, double turn_starti){
  desired_heading = angle;
  // Desired heading carries over the angle from one movement to another. That way, if the robot doesn't
  // finish a turn movement, it will still drive at the angle that was specified in the turn movement.
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    double error = reduce_negative_180_to_180(angle - get_absolute_heading());
    // Reducing the angle to a value between -180 and 180 degrees ensures that the robot always takes the 
    // shorter path when making a turn.
    double output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    pros::delay(10);
  }
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}

void Drive::drive_distance(double distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(double distance, double heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_kp, double drive_ki, double drive_kd, double drive_starti, double heading_kp, double heading_ki, double heading_kd, double heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  double start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at
  // and determines error relative to that value.
  double average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    double drive_error = distance+start_average_position-average_position;
    double heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    double drive_output = drivePID.compute(drive_error);
    double heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    pros::delay(10);
  }
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}

void Drive::left_swing_to_angle(double angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(double angle, double swing_max_voltage, double swing_settle_error, double swing_settle_time, double swing_timeout, double swing_kp, double swing_ki, double swing_kd, double swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    double error = reduce_negative_180_to_180(angle - get_absolute_heading());
    double output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    drive_with_voltage(output, 0);
    pros::delay(10);
  }
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}

void Drive::right_swing_to_angle(double angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(double angle, double swing_max_voltage, double swing_settle_error, double swing_settle_time, double swing_timeout, double swing_kp, double swing_ki, double swing_kd, double swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    double error = reduce_negative_180_to_180(angle - get_absolute_heading());
    double output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    drive_with_voltage(0, output);
    pros::delay(10);
  }
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}

double Drive::get_ForwardTracker_position(){
    return(get_right_position_in());
}



void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    pros::delay(5);
  }
}

void Drive::set_heading(double orientation_deg){
  inertial.set_heading(orientation_deg*gyro_scale/360.0);
}

void Drive::set_coordinates(double X_position, double Y_position, double orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
}

double Drive::get_X_position(){
  return(odom.X_position);
}

double Drive::get_Y_position(){
  return(odom.Y_position);
}

void Drive::drive_to_point(double X_position, double Y_position){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_kp, double drive_ki, double drive_kd, double drive_starti, double heading_kp, double heading_ki, double heading_kd, double heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(drivePID.is_settled() == false){
    double drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    // The drive error is just equal to the distance between the current and desired points.
    double heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
    double drive_output = drivePID.compute(drive_error);

    double heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    // The scale factor slows the drive down the more it's facing away from the desired point,
    // and that way the heading correction has time to catch up.
    heading_error = reduce_negative_90_to_90(heading_error);
    // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
    // to do so.
    double heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }
    // This if statement prevents the heading correction from acting up after the robot gets close
    // to being settled.

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    pros::delay(10);
  }
  desired_heading = get_absolute_heading();
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}

void Drive::turn_to_point(double X_position, double Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(double X_position, double Y_position, double extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(double X_position, double Y_position, double extra_angle_deg, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(double X_position, double Y_position, double extra_angle_deg, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout, double turn_kp, double turn_ki, double turn_kd, double turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    double error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    // Again, using atan2(x,y) puts 0 degrees on the positive Y axis.
    double output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    pros::delay(10);
  }
  desired_heading = get_absolute_heading();
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive_with_voltage(0, 0);
}



// The usercontrol functions use deadband=5 everywhere. This value pretty much gets the job done,
// but it can be changed with no repercussions.


int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}