/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:14:17 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 20:19:11
 */

#ifndef FRONTEND_GUIDED_MATCHER_H
#define FRONTEND_GUIDED_MATCHER_H

#include <memory>

#include "frame.h"
#include "landmark.h"
#include "pinhole_camera.h"

namespace lslam {

class GuidedMatcher {
public:
  GuidedMatcher() {};

  // Mutators
  void set_camera_model(std::shared_ptr<PinholeCamera>);
  void set_orb_extractor(std::shared_ptr<ORB_SLAM2::ORBextractor>);
  // 3d-2d matcher
  // Project Landmarks into the current frame, and search matches
  int ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::vector<std::shared_ptr<Landmark>> landmarks);

  int ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::shared_ptr<Frame> last_frame);
private:
  std::shared_ptr<PinholeCamera> camera_model_;
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  
  // Scale pyramid info.
  int scale_levels_;
  float scale_factor_;
  float log_scale_factor_;//
  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> level_sigma2_;
  std::vector<float> inv_level_sigma2_;
};

} // namespace lslam

#endif // FRONTEND_GUIDED_MATCHER_H
 