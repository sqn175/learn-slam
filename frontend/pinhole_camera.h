/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef FRONTEND_PINHOLE_CAMERA_H_
#define FRONTEND_PINHOLE_CAMERA_H_

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

#include "parameters.h"

namespace lslam {

class PinholeCamera{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PinholeCamera() { }
  PinholeCamera(PinholeCameraParameters params);
  
  ~PinholeCamera() { }
  
  // construct pinhole_camera
  void init(PinholeCameraParameters params);

  // Get calibration matrix/intrinsici matrix
  cv::Mat K() const; 
  
private:
  int img_width_; // image width in pixels
  int img_height_; // image height in pixels
  
  double fu_; // camera focal lengths
  double fv_; 
  double cu_; // optical centers expressed in pixels coordinates
  double cv_;
  
  std::string distortion_type_; // distortion type
  Eigen::Matrix<double, 4, 1> radtan_distortion_coef_; // radial and tangential distortion parameters
  double k1_; // radial parameter 1
  double k2_; // radial parameter 2
  double p1_; // tangential parameter 1
  double p2_; // tangential parameter 2
  
};

} // namespace lslam

#endif // FRONTEND_PINHOLE_CAMERA_H_
