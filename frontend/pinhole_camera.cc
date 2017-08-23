/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#include "pinhole_camera.h"

namespace lslam{

PinholeCamera::PinholeCamera(PinholeCameraParameters params) {
  img_width_ = params.image_dimension[0];
  img_height_ = params.image_dimension[1];
  
  fu_ = params.focal_length[0];
  fv_ = params.focal_length[1];
  cu_ = params.principal_point[0];
  cv_ = params.principal_point[1];
  
  distortion_type_ = params.distortion_type;
  radtan_distortion_coef_ = params.distortion_coef;
  k1_ = params.distortion_coef[0];
  k2_ = params.distortion_coef[1];
  p1_ = params.distortion_coef[0];
  p2_ = params.distortion_coef[1];
}

cv::Mat PinholeCamera::K() const {
  return cv::Mat_<double>(3,3) << fu_, 0, cu_, 0, fv_, cv_, 0, 0, 1;
}

} // namespace lslam
