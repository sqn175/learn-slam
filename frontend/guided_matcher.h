/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:14:17 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 19:49:46
 */

#ifndef FRONTEND_GUIDED_MATCHER_H
#define FRONTEND_GUIDED_MATCHER_H

#include <memory>
#include <set>

#include "../3rdparty/ORB_SLAM2_modified/ORBextractor.h"
//#include "frame.h"
#include "pinhole_camera.h"

namespace lslam {

class Frame;
class KeyFrame;
class MapPoint;

class GuidedMatcher {
public:
  enum RadiusFlag {
    kRadiusFromViewCosine = 0,
    kRadiusFromOctave,
  };
public:
  GuidedMatcher() {};
  GuidedMatcher(std::shared_ptr<PinholeCamera> camera_model,
                std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor);

  // 2d-2d matcher
  void SetupGuided2D2DMatcher(std::shared_ptr<Frame> query_frame);
  std::vector<cv::DMatch> Guided2D2DMatcher(std::shared_ptr<Frame> train_frame, 
                                            const int r, const float dist_th,
                                            const float ratio, 
                                            const bool check_rotation );
  // 3d-2d matcher
  std::vector<cv::DMatch> ProjectionGuided3D2DMatcher(std::vector<std::shared_ptr<MapPoint>> query_mappoints, 
                                                      std::shared_ptr<Frame> train_frame, 
                                                      const double radius_factor, RadiusFlag flag,
                                                      const bool check_proj_error,
                                                      const double dist_th, 
                                                      const float ratio = 0);

  std::vector<cv::DMatch> ProjectionGuided3D2DMatcher(std::vector<std::shared_ptr<MapPoint>> query_mappoints, 
                                                      std::shared_ptr<Frame> train_frame, 
                                                      const double radius_factor, RadiusFlag flag,
                                                      const bool check_proj_error,
                                                      const double dist_th, 
                                                      const float ratio,
                                                      std::vector<size_t>& query_indices); 

  std::vector<cv::DMatch> ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> query_frame, 
                                                      std::shared_ptr<Frame> train_frame, 
                                                      const double radius_factor, RadiusFlag flag,
                                                      const bool check_proj_error,
                                                      const double dist_th, 
                                                      const bool check_rotation,
                                                      const float ratio = 0);                                     

  std::vector<cv::DMatch> DbowGuided2D2DMatcher(std::shared_ptr<KeyFrame> query_frame, 
                                                std::shared_ptr<Frame> train_frame, 
                                                const double dist_th, 
                                                const bool check_rotation,
                                                const float ratio=0.7);

  std::vector<cv::DMatch> DbowAndEpipolarGuided2D2DMatcher(std::shared_ptr<KeyFrame> query_frame, 
                                                std::shared_ptr<KeyFrame> train_frame, 
                                                const double dist_th, 
                                                const bool check_rotation,
                                                const float ratio=0);

public:
  static const int TH_LOW;
  static const int TH_HIGH;

private:

  std::vector<cv::DMatch> Matcher(const cv::Mat& query_descriptors, const std::vector<size_t> query_indices,
                                  const cv::Mat& train_descriptors, const std::vector<std::vector<size_t>> guided_train_indices,
                                  const float dist_th, 
                                  const float ratio = -1);

  std::vector<cv::DMatch> CheckRotation(std::shared_ptr<Frame> query_frame, std::shared_ptr<Frame> train_frame, std::vector<cv::DMatch> matches);
  int DescriptorDist(const cv::Mat& a, const cv::Mat& b);
  void ComputeThreeMaxima(const std::vector<std::vector<int>>& histo, const int L, int& ind1, int& ind2, int& ind3);
private:
  std::shared_ptr<PinholeCamera> camera_model_;
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  
  // Scale pyramid info.
  int max_level_;
  int scale_levels_;
  float scale_factor_;
  float log_scale_factor_;//
  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> level_sigma2_;
  std::vector<float> inv_level_sigma2_;

  // 2d-2d matcher variables, only modified in frontend thread, it's thread safe
  std::shared_ptr<Frame> init_query_frame_; // Initial reference frame
  std::vector<cv::Point2f> guided_2d_pts_; // We search around guided_2d_pts in train frame 
  cv::Mat init_query_descriptors_;
  std::vector<size_t> init_query_indices_;
};

} // namespace lslam

#endif // FRONTEND_GUIDED_MATCHER_H
 