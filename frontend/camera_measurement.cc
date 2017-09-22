/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "camera_measurement.h"

#include "my_assert.h"

namespace lslam {

CameraMeasurement::CameraMeasurement(unsigned long timestamp, unsigned long id, const cv::Mat& image)
  : timestamp_(timestamp)
  , id_(id) 
  , image_(image){

}

CameraMeasurement::CameraMeasurement(const CameraMeasurement& cm)
  : id_(cm.id_)
  , image_(cm.image_)
  , keypoints_(cm.keypoints_)
  , descriptors_(cm.descriptors_) {
}

void CameraMeasurement::ExtractOrb(std::shared_ptr<ORB_SLAM2::ORBextractor> extractor) {
  (*extractor)(image_, cv::Mat(), keypoints_, descriptors_);
}

std::vector<cv::KeyPoint> CameraMeasurement::keypoints() const {
  return keypoints_;
}

cv::Mat CameraMeasurement::descriptors() const {
  return descriptors_;
}

void CameraMeasurement::SetPose(cv::Mat T_cw) {
  T_cw_ = T_cw.clone();
  R_cw_ = T_cw.rowRange(0,3).colRange(0,3);
  t_cw_ = T_cw.rowRange(0,3).col(3);
  o_w_ = -R_cw_.t() * t_cw_;
  
  T_wc_ = cv::Mat::eye(4,4,T_cw_.type());
  cv::Mat R_wc_ = R_cw_.t();
  R_wc_.copyTo(T_wc_.rowRange(0,3).colRange(0,3));
  o_w_.copyTo(T_wc_.rowRange(0,3).col(3));
}

cv::Mat CameraMeasurement::Tcw() const {
  return T_cw_.clone();
}

cv::Mat CameraMeasurement::Twc() const {
  return T_wc_.clone();
}

} // namespace lslam
