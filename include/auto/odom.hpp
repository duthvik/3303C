#include "main.h"

class Odom
{
private:
  double ForwardTracker_center_distance;
  double SidewaysTracker_center_distance;
  double ForwardTracker_position;
  double SideWaysTracker_position;
public:
  double X_position;
  double Y_position;
  double orientation_deg;
  void set_position(double X_position, double Y_position, double orientation_deg, double ForwardTracker_position, double SidewaysTracker_position);
  void update_position(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg);
  void set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance);
};