/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef FRONTEND_PINHOLE_CAMERA_H_
#define FRONTEND_PINHOLE_CAMERA_H_

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "parameters.h"

namespace lslam {

class PinholeCamera{
public:
  PinholeCamera() { }
  PinholeCamera(PinholeCameraParameters params);
  
  ~PinholeCamera() { }
  
  // construct pinhole_camera
  void init(const PinholeCameraParameters& params);

  // Get calibration matrix/intrinsici matrix
  cv::Mat K() const; 

  // Get distortion coefficients
  cv::Mat DistCoeffs() const;

  std::string DistortionType() const;

  cv::Mat image_bounds() const;
  // Projection functions
  // Project camera coordinate 3d point to image coordinate 2d point 
  cv::Mat Project(cv::Mat& pt3d);

private:
  // Compute undistorted image bounds 
  void GetImageBounds();

private:
  int img_width_; // image width in pixels
  int img_height_; // image height in pixels

  // Image bounds after undistortion
  cv::Mat image_bounds_;
  
  cv::Mat K_;
  double fu_; // camera focal lengths
  double fv_; 
  double cu_; // optical centers expressed in pixels coordinates
  double cv_;
  
  std::string distortion_type_; // distortion type
  // distortion_coeffs_= [k1,k2,p1,p2]
  cv::Mat distortion_coeffs_; // radial and tangential distortion parameters
  double k1_; // radial parameter 1
  double k2_; // radial parameter 2
  double p1_; // tangential parameter 1
  double p2_; // tangential parameter 2
  
};

} // namespace lslam

#endif // FRONTEND_PINHOLE_CAMERA_H_
