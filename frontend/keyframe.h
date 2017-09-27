/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_KEYFRAME_H
#define FRONTEND_KEYFRAME_H

#include <memory>

#include "frame.h"

namespace lslam {

class Frame;

class KeyFrame{
public:
  KeyFrame(std::shared_ptr<Frame> camera_measurement);
  ~KeyFrame() { }
  
  std::shared_ptr<Frame> camera_measurement() const;
  
private:
  std::shared_ptr<Frame> camera_measurement_; // Set camera_measurement_ as keyframe
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
