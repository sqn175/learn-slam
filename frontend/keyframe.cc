/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "keyframe.h"

namespace lslam {

KeyFrame::KeyFrame(std::shared_ptr<CameraMeasurement> camera_measurement) 
  : camera_measurement_(camera_measurement) {
}

std::shared_ptr<CameraMeasurement> KeyFrame::camera_measurement() const {
  return camera_measurement_;
}

} // namespace lslam
