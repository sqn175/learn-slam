/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "frame.h"

#include "my_assert.h"

namespace lslam {

void CameraMeasurement::ExtractORB() {
  ASLAM_ASSERT_TRUE(orb_extractor_, "");
  (*orb_extractor_)( image_, cv::Mat(), keypoints_, descriptors_);
}

} // namespace lslam
