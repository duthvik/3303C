#include "main.h"

double reduce_0_to_360(double angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

double reduce_negative_180_to_180(double angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

double reduce_negative_90_to_90(double angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

double to_rad(double angle_deg){
  return(angle_deg/(180.0/M_PI));
}

double to_deg(double angle_rad){
  return(angle_rad*(180.0/M_PI));
}

double clamp(double input, double min, double max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

double to_volt(double percent){
  return(percent*12.0/100.0);
}

int to_port(int port){
  if(port>8){
    return(0);
  }
  return(port-1);
} // To_port just keeps a number over 7 from being passed as a threewire port.

double deadband(double input, double width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}