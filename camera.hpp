#ifndef CAMERA_HPP
#define CAMERA_HPP


#include "libpixyusb2.h"   // bring Pixy2 into scope properly

int  get_heading_error(Pixy2 &pixy);
void get_line_features(Pixy2 &pixy);
int setup_camera(Pixy2 &pixy);

#endif // CAMERA_HPP