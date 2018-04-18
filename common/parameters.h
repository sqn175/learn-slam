/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef COMMON_PARAMETERS_H_
#define COMMON_PARAMETERS_H_

#include <string>
#include <vector>

namespace lslam {

struct PinholeCameraParameters{
  int img_width; // image width in pixels
  int img_height; // image height in pixels
  
  double fx; // camera focal lengths
  double fy; 
  double cx; // optical centers expressed in pixels coordinates
  double cy;
  std::string distortion_type;      // distortion type
  
  double k1; // radial parameter 1
  double k2; // radial parameter 2
  double p1; // tangential parameter 1
  double p2; // tangential parameter 2

  int frame_rate;          // frame rate
};

struct FeatureParameters {
  int n_feat; // features extracted per image
  double scale_factor; 
  int levels; 
  int init_th_Fast;
  int min_th_Fast;

  std::vector<float> scale_factors;
  std::vector<float> inv_scale_factors;    
  std::vector<float> level_sigma2;
  std::vector<float> inv_level_sigma2;
};

} // namespace LSLAM

#endif  //COMMON_PARAMETERS_H_
