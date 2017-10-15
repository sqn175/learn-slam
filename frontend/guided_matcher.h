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

  // 2d-2d matcher
  void SetupGuided2D2DMatcher(std::shared_ptr<Frame> init_frame);
  int Guided2D2DMatcher(std::shared_ptr<Frame> train_frame, std::vector<cv::DMatch>& matches, 
    const int radius,const float dist_th, const bool use_ratio_test, const float ratio, const bool check_rotation );
  // 3d-2d matcher
  // Project Landmarks into the current frame, and search matches
  int ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::vector<std::shared_ptr<Landmark>> landmarks);

  int ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::shared_ptr<Frame> last_frame, const double th, const bool check_ori);

public:
  static const int TH_LOW;
  static const int TH_HIGH;

private:

  // For every descriptor of query_descriptors, we have a corresponding guided_search_range in train_frame.
  // According to this guided_search_range, we can search fast. If guided_search_range is null, we skip this descriptor.
  // guided_search_octaves: [min_octave, max_octave], included
  int Matcher(const cv::Mat& query_descriptors, const std::shared_ptr<Frame> train_frame, 
    const std::vector<cv::Mat>& guided_search_ranges,
    const std::vector<std::pair<int, int>>& guided_search_octaves,
    std::vector<cv::DMatch>& matches,
    const float dist_th, const bool use_ratio_test, const float ratio);
  int DescriptorDist(const cv::Mat& a, const cv::Mat& b);
  void ComputeThreeMaxima(const std::vector<std::vector<int>>& histo, const int L, int& ind1, int& ind2, int& ind3);
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

  // 2d-2d matcher variables
  std::shared_ptr<Frame> init_query_frame_; // Initial reference frame
  std::vector<cv::Point2f> guided_2d_pts_; // We search around guided_2d_pts in train frame 
  cv::Mat init_query_descriptors_;
};

} // namespace lslam

#endif // FRONTEND_GUIDED_MATCHER_H
 