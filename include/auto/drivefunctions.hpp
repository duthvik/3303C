#include "main.h"
#include "okapi/impl/device/rotarysensor/IMU.hpp"

#pragma once






class Drive
{
private:
  double wheel_diameter;
  double wheel_ratio;
  double gyro_scale;
  double drive_in_to_deg_ratio;
  double ForwardTracker_center_distance;
  double ForwardTracker_diameter;
  double ForwardTracker_in_to_deg_ratio;
  double SidewaysTracker_center_distance;
  double SidewaysTracker_diameter;
  double SidewaysTracker_in_to_deg_ratio;
  

public: 


  double turn_max_voltage;
  double turn_kp;
  double turn_ki;
  double turn_kd;
  double turn_starti;

  double turn_settle_error;
  double turn_settle_time;
  double turn_timeout;

  double drive_max_voltage;
  double drive_kp;
  double drive_ki;
  double drive_kd;
  double drive_starti;

  double drive_settle_error;
  double drive_settle_time;
  double drive_timeout;

  double heading_max_voltage;
  double heading_kp;
  double heading_ki;
  double heading_kd;
  double heading_starti;

  double swing_max_voltage;
  double swing_kp;
  double swing_ki;
  double swing_kd;
  double swing_starti;

  double swing_settle_error;
  double swing_settle_time;
  double swing_timeout;
  
  double desired_heading;

  Drive(double wheel_diameter, double wheel_ratio, double gyro_scale, double ForwardTracker_diameter, double ForwardTracker_center_distance, double SidewaysTracker_diameter, double SidewaysTracker_center_distance);

  void drive_with_voltage(double leftVoltage, double rightVoltage);

  double get_absolute_heading();

  double get_left_position_in();

  double get_right_position_in();

  void set_turn_constants(double turn_max_voltage, double turn_kp, double turn_ki, double turn_kd, double turn_starti); 
  void set_drive_constants(double drive_max_voltage, double drive_kp, double drive_ki, double drive_kd, double drive_starti);
  void set_heading_constants(double heading_max_voltage, double heading_kp, double heading_ki, double heading_kd, double heading_starti);
  void set_swing_constants(double swing_max_voltage, double swing_kp, double swing_ki, double swing_kd, double swing_starti);

  void set_turn_exit_conditions(double turn_settle_error, double turn_settle_time, double turn_timeout);
  void set_drive_exit_conditions(double drive_settle_error, double drive_settle_time, double drive_timeout);
  void set_swing_exit_conditions(double swing_settle_error, double swing_settle_time, double swing_timeout);

  void turn_to_angle(double angle);
  void turn_to_angle(double angle, double turn_max_voltage);
  void turn_to_angle(double angle, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout);
  void turn_to_angle(double angle, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout, double turn_kp, double turn_ki, double turn_kd, double turn_starti);

  void drive_distance(double distance);
  void drive_distance(double distance, double heading);
  void drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage);
  void drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout);
  void drive_distance(double distance, double heading, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_kp, double drive_ki, double drive_kd, double drive_starti, double heading_kp, double heading_ki, double heading_kd, double heading_starti);

  void left_swing_to_angle(double angle);
  void left_swing_to_angle(double angle, double swing_max_voltage, double swing_settle_error, double swing_settle_time, double swing_timeout, double swing_kp, double swing_ki, double swing_kd, double swing_starti);
  void right_swing_to_angle(double angle);
  void right_swing_to_angle(double angle, double swing_max_voltage, double swing_settle_error, double swing_settle_time, double swing_timeout, double swing_kp, double swing_ki, double swing_kd, double swing_starti);
  
  Odom odom;
  double get_ForwardTracker_position();
  double get_SidewaysTracker_position();
  void set_coordinates(double X_position, double Y_position, double orientation_deg);
  void set_heading(double orientation_deg);
  void position_track();
  static int position_track_task();
  double get_X_position();
  double get_Y_position();

  void drive_to_point(double X_position, double Y_position);
  void drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage);
  void drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout);
  void drive_to_point(double X_position, double Y_position, double drive_max_voltage, double heading_max_voltage, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_kp, double drive_ki, double drive_kd, double drive_starti, double heading_kp, double heading_ki, double heading_kd, double heading_starti);
  
  void turn_to_point(double X_position, double Y_position);
  void turn_to_point(double X_position, double Y_position, double extra_angle_deg);
  void turn_to_point(double X_position, double Y_position, double extra_angle_deg, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout);
  void turn_to_point(double X_position, double Y_position, double extra_angle_deg, double turn_max_voltage, double turn_settle_error, double turn_settle_time, double turn_timeout, double turn_kp, double turn_ki, double turn_kd, double turn_starti);


};