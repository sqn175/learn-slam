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

  fx_ = params.fx;
  fy_ = params.fy;
  cx_ = params.cx;
  cy_ = params.cy;
  K_ = (cv::Mat_<double>(3,3)<<fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);

  distortion_type_ = params.distortion_type;

  k1_ = params.k1;
  k2_ = params.k2;
  p1_ = params.p1;
  p2_ = params.p2;

  distortion_coeffs_ = (cv::Mat_<double>(4,1)<<k1_, k2_, p1_, p2_);

  SetImageBounds();
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
  const double u = fx_*pt3d.at<double>(0)*z_inv + cx_;
  const double v = fy_*pt3d.at<double>(1)*z_inv + cy_;
  return (cv::Mat_<double>(2,1) << u,v);
}

void PinholeCamera::SetImageBounds() {
  if (distortion_type_.compare("radialtangential") == 0) {

    // topleft, bottomleft, topright, bottomright
    cv::Mat mat = (cv::Mat_<float>(4,2)<</* image topleft */   0.0, 0.0, 
                                         /* image topright */  img_width_-1.0, 0.0, 
                                         /* image bottomleft*/ 0.0, img_height_-1.0,
                                         /* image bottomright*/img_width_-1.0,img_height_-1.0);

    // Undistort corners
    mat = mat.reshape(2);
    cv::undistortPoints(mat,mat,K_,distortion_coeffs_,cv::noArray(),K_);
    mat = mat.reshape(1);

    // bounds
    double min_x = std::min(mat.at<float>(0,0),mat.at<float>(2,0)); // min_x <= topleft.x && min_x <= bottomleft.x
    double min_y = std::min(mat.at<float>(0,1),mat.at<float>(1,1)); // min_y <= topleft.y && min_y <= topright.y
    double max_x = std::max(mat.at<float>(1,0),mat.at<float>(3,0)); // max_x >= topright.x && max_x >= bottomright.x
    double max_y = std::max(mat.at<float>(2,1),mat.at<float>(3,1)); // max_y >= bottomleft.y && max_y >= bottomright.y

    image_bounds_ = (cv::Mat_<double>(2,2)<<min_x,min_y,max_x,max_y);
  }
  else 
    image_bounds_ = (cv::Mat_<double>(2,2)<<0.0,0.0,img_width_-1,img_height_-1);
}

} // namespace lslam
