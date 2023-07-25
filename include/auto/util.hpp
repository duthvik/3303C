#include "main.h"

double reduce_0_to_360(double angle);

double reduce_negative_180_to_180(double angle);

double reduce_negative_90_to_90(double angle);

double to_rad(double angle_deg);

double to_deg(double angle_rad);

double clamp(double input, double min, double max);

bool is_reversed(double input);

double to_volt(double percent);

int to_port(int port);

double deadband(double input, double width);