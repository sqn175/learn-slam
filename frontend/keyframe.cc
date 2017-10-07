/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "keyframe.h"

namespace lslam {

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame) 
  : frame_(frame) {
}

std::shared_ptr<Frame> KeyFrame::frame() const {
  return frame_;
}

} // namespace lslam
