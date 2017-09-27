/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#include "pinhole_camera.h"

namespace lslam{

PinholeCamera::PinholeCamera(PinholeCameraParameters params) {
  init(params);
}

void PinholeCamera::init(const PinholeCameraParameters& params) {
  img_width_ = params.img_width;
  img_height_ = params.img_height;

  fu_ = params.fu;
  fv_ = params.fv;
  cu_ = params.cu;
  cv_ = params.cv;
  K_ = cv::Mat_<double>(3,3)<<fu_, 0, cu_, 0, fv_, cv_, 0, 0, 1;

  distortion_type_ = params.distortion_type;

  k1_ = params.k1;
  k2_ = params.k2;
  p1_ = params.p1;
  p2_ = params.p2;

  distortion_coeffs_ = cv::Mat_<double>(4,1)<<k1_, k2_, p1_, p2_;
}

cv::Mat PinholeCamera::K() const {
  return K_;
}

cv::Mat PinholeCamera::DistCoeffs() const {
  return distortion_coeffs_;
}

std::string PinholeCamera::DistortionType() const {
  return distortion_type_;
}

cv::Mat PinholeCamera::image_bounds() const {
  return image_bounds_;
}

cv::Mat PinholeCamera::Project(cv::Mat& pt3d) {
  CHECK(pt3d.cols == 1 && pt3d.rows == 3) << "Invalid 3d point.";
  // Check positive depth?
  const double pz_c = pt3d.at<double>(2);
  if (pz_c == 0.0) {
    return cv::Mat();
  }

  // Project to image coordinate and check it is inside
  //  |u| |fu 0 cu||x|
  // s|v|=|0 fv cv||y|
  //  |1| |0  0  1||z|
  const double z_inv = 1.0/pz_c;
  const double u = fu_*pt3d.at<double>(0)*z_inv + cu_;
  const double v = fv_*pt3d.at<double>(1)*z_inv + cv_;
  return cv::Mat_<double>(2,1) << u,v;
}

void PinholeCamera::GetImageBounds() {
  if (distortion_type_.compare("radialtangential") == 0) {
    double corners[8] = {0.0,0.0, img_height_-1,0.0, 0.0,img_width_-1, img_height_-1,img_width_-1};
    cv::Mat mat(4,2,CV_64F,corners);

    // Undistort corners
    mat = mat.reshape(2);
    cv::undistortPoints(mat,mat,K_,distortion_coeffs_,cv::Mat(),K_);
    mat = mat.reshape(1);

    // bounds
    double min_u = std::min(mat.at<float>(0,0),mat.at<float>(2,0));
    double min_v = std::min(mat.at<float>(0,1),mat.at<float>(1,1));
    double max_u = std::max(mat.at<float>(1,0),mat.at<float>(3,0));
    double max_v = std::max(mat.at<float>(2,1),mat.at<float>(3,1));

    image_bounds_ = cv::Mat_<double>(4,1)<<min_u,min_v,max_u,max_v;
  }
  else 
    image_bounds_ = cv::Mat_<double>(4,1)<<0.0,0.0,img_height_-1,img_width_-1;
}

} // namespace lslam
