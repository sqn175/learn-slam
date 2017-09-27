/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:19:52 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 21:01:20
 */

#include "guided_matcher.h"

namespace lslam {

void GuidedMatcher::set_camera_model(std::shared_ptr<PinholeCamera> camera_model) {
  camera_model_ = camera_model;
}

void GuidedMatcher::set_orb_extractor( std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor) {
  orb_extractor_ = orb_extractor;
  scale_levels_ = orb_extractor->GetLevels();
  scale_factor_ = orb_extractor->GetScaleFactor();
  log_scale_factor_ = std::log(scale_factor_);
  scale_factors_ = orb_extractor->GetScaleFactors();
  inv_scale_factors_ = orb_extractor->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor->GetInverseScaleSigmaSquares();
}
  
int GuidedMatcher::ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::vector<std::shared_ptr<Landmark>> landmarks) {
  int n_matches = 0;

  for (auto& landmark : landmarks) {
    // Check if this landmark is projection valid
    
  }
  return n_matches;
}

int GuidedMatcher::ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::shared_ptr<Frame> last_frame) {
  int n_matches = 0;

  int th = 15;

  std::vector<std::shared_ptr<Landmark>> landmarks = last_frame->landmarks();
  std::vector<cv::KeyPoint> last_keypoints = last_frame->keypoints();
  for (int i = 0; i < landmarks.size(); ++i) {
    auto landmark = landmarks[i];
    if (landmark) {
      // Project landmark to current frame, projection result is uv
      cv::Mat p_uv;
      if (landmark->IsProjectable(cur_frame, camera_model_, p_uv)) {
        int last_octave = last_keypoints[i].octave;
        // Search in a window, size depends on scale
        double r = th*scale_factors_[last_octave];
        
        // We search
        double u = p_uv.at<double>(0);
        double v = p_uv.at<double>(1);
        cv::Mat search_bounds(4,1,CV_64F);
        search_bounds.at<double>(0) = u-r;
        search_bounds.at<double>(1) = v-r;
        search_bounds.at<double>(2) = u+r;
        search_bounds.at<double>(3) = v+r;

        std::vector<size_t> indices = cur_frame->range_searcher()->PointsInRange(search_bounds);
        if (indices.empty())  continue;

        
      }
    }
  }
}
} // namespace lslam