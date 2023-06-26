#include "main.h"

class Odom{
    private:
        double FTrackerCenterDist;
        double STrackerCenterDist;
        double FTrackerPos;
        double STrackerPods;
    
    public:
        double Xpos;
        double Ypos;
        double orientation_deg;
        void setPosition(double X_position, double Y_position, double orientation_deg, double ForwardTracker_position, double SidewaysTracker_position);
        void updatePosition(double ForwardTracker_position, double SidewaysTracker_position, double orientation_deg);
        void set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance);        
};