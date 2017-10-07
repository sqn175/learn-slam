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
  KeyFrame(std::shared_ptr<Frame> frame);
  ~KeyFrame() { }
  
  std::shared_ptr<Frame> frame() const;
  
private:
  std::shared_ptr<Frame> frame_; // Set frame_ as keyframe
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
