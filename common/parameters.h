/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef COMMON_PARAMETERS_H_
#define COMMON_PARAMETERS_H_

#include <string>

namespace lslam {

struct PinholeCameraParameters{
  int img_width; // image width in pixels
  int img_height; // image height in pixels
  
  double fu; // camera focal lengths
  double fv; 
  double cu; // optical centers expressed in pixels coordinates
  double cv;
  std::string distortion_type;      // distortion type
  
  double k1; // radial parameter 1
  double k2; // radial parameter 2
  double p1; // tangential parameter 1
  double p2; // tangential parameter 2

  int frame_rate;          // frame rate
};
} // namespace LSLAM

#endif  //COMMON_PARAMETERS_H_
