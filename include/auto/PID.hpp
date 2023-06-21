#include "main.h"

class PID{
    public:
        double error =0,
        kp=0,
        ki=0,
        kd=0,
        starti=0,
        settleError=0,
        settleTime=0,
        timeout=0,
        accumError=0,
        prevError=0,
        output=0,
        settlingTime=0,
        runningTime=0;

        PID(double error, double kp, double ki, double kd, double starti, double settleError, double settleTime, double timeout);
        PID(double error, double kp, double ki, double kd, double starti);

        double compute(double error);

        bool is_settled();

};