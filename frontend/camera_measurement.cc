/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "camera_measurement.h"

#include "my_assert.h"

namespace lslam {

CameraMeasurement::CameraMeasurement(unsigned long id, const cv::Mat& image,
      std::shared_ptr<const PinholeCamera>& camera_model,
      std::shared_ptr<ORB_SLAM2::ORBextractor>& orb_extractor)
  : id_(id) 
  , image_(image)
  , camera_model_(camera_model)
  , orb_extractor_(orb_extractor) {
  // Extract ORB 
  ExtractORB();
}

CameraMeasurement::CameraMeasurement(const CameraMeasurement& cm)
  : id_(cm.id_)
  , image_(cm.image_)
  , camera_model_(cm.camera_model_)
  , orb_extractor_(cm.orb_extractor_)
  , keypoints_(cm.keypoints_)
  , descriptors_(cm.descriptors_) {
}

std::vector<cv::KeyPoint> CameraMeasurement::keypoints() const {
  return keypoints_;
}

cv::Mat CameraMeasurement::descriptors() const {
  return descriptors_;
}

std::shared_ptr<const PinholeCamera> CameraMeasurement::camera_model() const {
  return camera_model_;
}

void CameraMeasurement::ExtractORB() {
  ASLAM_ASSERT_TRUE(orb_extractor_, "");
  (*orb_extractor_)( image_, cv::Mat(), keypoints_, descriptors_);
}

void CameraMeasurement::SetPose(cv::Mat T_cw) {
  T_cw_ = T_cw.clone();
  R_cw_ = T_cw.rowRange(0,3).colRange(0,3);
  t_cw_ = T_cw.rowRange(0,3).col(3);
  o_w_ = -R_cw_.t() * t_cw_;
}

} // namespace lslam
