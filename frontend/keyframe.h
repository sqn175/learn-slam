/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_KEYFRAME_H
#define FRONTEND_KEYFRAME_H

#include <memory>

#include "camera_measurement.h"

namespace lslam {

class KeyFrame{
public:
  KeyFrame(std::shared_ptr<CameraMeasurement> camera_measurement);
  ~KeyFrame() { }
  
  std::shared_ptr<CameraMeasurement> camera_measurement() const;
  
private:
  std::shared_ptr<CameraMeasurement> camera_measurement_; // Set camera_measurement_ as keyframe
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
