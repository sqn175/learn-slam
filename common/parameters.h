/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef COMMON_PARAMETERS_H_
#define COMMON_PARAMETERS_H_

#include <string>

#include <Eigen/Core>

namespace lslam {

struct PinholeCameraParameters{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d image_dimension; // image dimension in pixels, width * height
  Eigen::Vector2d focal_length;     // focal length
  Eigen::Vector2d principal_point;  // principal point, optical centers expressed in pixels coordinates
  Eigen::VectorXd distortion_coef;  // distortion coefficients
  std::string distortion_type;      // distortion type
  int frame_rate;          // frame rate
};
} // namespace LSLAM

#endif  //COMMON_PARAMETERS_H_
