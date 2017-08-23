/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "camera_measurement.h"

#include "my_assert.h"

namespace lslam {

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

} // namespace lslam
