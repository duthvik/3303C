#include "main.h"


void Odom::set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance){
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

void Odom::set_position(double X_position, double Y_position, double orientation_deg, double ForwardTracker_position, double SidewaysTracker_position){
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_deg = orientation_deg;
}

void Odom::update_position(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg){
  // this-> always refers to the old version of the variable, so subtracting this->x from x gives delta x.
  double Forward_delta = ForwardTracker_position-this->ForwardTracker_position;
  double Sideways_delta = SidewaysTracker_position-this->SideWaysTracker_position;
  this->ForwardTracker_position=ForwardTracker_position;
  this->SideWaysTracker_position=SidewaysTracker_position;
  double orientation_rad = to_rad(orientation_deg);
  double prev_orientation_rad = to_rad(this->orientation_deg);
  double orientation_delta_rad = orientation_rad-prev_orientation_rad;
  this->orientation_deg=orientation_deg;

  double local_X_position;
  double local_Y_position;

  // All of the following lines are pretty well documented in 5225A's Into to Position Tracking 
  // Document at http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf 

  if (orientation_delta_rad == 0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((Sideways_delta/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  double local_polar_angle;
  double local_polar_length;

  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  double global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2);

  double X_position_delta = local_polar_length*cos(global_polar_angle); 
  double Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;
}