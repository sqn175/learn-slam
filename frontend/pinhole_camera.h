/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef FRONTEND_PINHOLE_CAMERA_H_
#define FRONTEND_PINHOLE_CAMERA_H_

#include <eigen3/Eigen/Core>

namespace lslam {

struct PinholeCameraParams{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int img_width; // image width in pixels
  int img_height; // image height in pixels
  Eigen::VectorXd distortion_coef; // radial and tangential distortion coefficients (k1,k2,p1,p2,k3)
  double fx; // camera focal lengths
  double fy; 
  double px; // optical centers expressed in pixels coordinates
  double py;
};

} // namespace lslam

#endif // FRONTEND_PINHOLE_CAMERA_H_
