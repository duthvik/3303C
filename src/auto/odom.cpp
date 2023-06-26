#include "main.h"

void Odom::setPosition(double X_position, double Y_position, double orientation_deg, double ForwardTracker_position, double SidewaysTracker_position)
{
    Xpos = X_position;
    Ypos = Y_position;
    this->orientation_deg = orientation_deg;
    FTrackerPos = ForwardTracker_position;
    STrackerPods = SidewaysTracker_position;
}

void Odom::set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance)
{
    FTrackerCenterDist = ForwardTracker_center_distance;
    STrackerCenterDist = SidewaysTracker_center_distance;
}

void Odom::updatePosition(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg)
{
    
}