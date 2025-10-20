#ifndef CAMERA_HPP
#define CAMERA_HPP


#include "libpixyusb2.h"   // bring Pixy2 into scope properly
#include <cmath>
#include <numbers>    // for std::numbers::pi_v<float>
#include <iostream>


int  get_heading_error(Pixy2 &pixy);
void get_line_features(Pixy2 &pixy);
int setup_camera(Pixy2 &pixy);
int get_theta_deg(Pixy2& pixy);

#endif // CAMERA_HPP