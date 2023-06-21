#include "main.h"
#include <cmath>

PID::PID(double error, double kp, double ki, double kd, double starti):
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti)
{};

PID::PID(double error, double kp, double ki, double kd, double starti, double settleError, double settleTime, double timeout):
    error(error),
    kp(kp),
    ki(ki),
    kd(kd),
    starti(starti),
    settleError(settleError),
    settleTime(settleTime),
    timeout(timeout)
{};

double PID::compute(double error)
{
  if(fabs(error) < starti) accumError+=error;

  if ((error>0 && prevError<0)||(error<0 && prevError>0)) accumError = 0; 

  output = kp*error + ki*accumError + kd*(error-prevError);

  prevError=error;

  if(fabs(error)<settleError){
    settlingTime+=10;
  } 
  
  else {
    settlingTime = 0;
  }

  runningTime+=10;

  return output;
}

bool PID::is_settled(){
  if (runningTime>timeout && timeout != 0){
    return(true);
  }
  if (settlingTime>settleTime){
    return(true);
  }
  return(false);
}
