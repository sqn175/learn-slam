/*
 * @Author: Shi Qin 
 * @Date: 2017-10-25 09:58:15 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 20:35:09
 */

#include "mapper.h"

#include "keyframe.h"
#include "mappoint.h"
#include "map.h"
#include "cv.h"
#include "guided_matcher.h"

namespace lslam {

Mapper::Mapper(std::shared_ptr<Map> map,
                std::shared_ptr<GuidedMatcher> guided_matcher) 
  : map_(map)
  , guided_matcher_(guided_matcher) {
                  
}

void Mapper::Process(std::shared_ptr<KeyFrame> keyframe) {
  InsertKeyFrame(keyframe);
  CullMapPoints();
  TriangulateNewMapPoints(keyframe);
}

void Mapper::InsertKeyFrame(std::shared_ptr<KeyFrame> keyframe){
  // Compute Bow
  keyframe->ComputeBoW();

  // Associate the keyframe to mappoints by adding observation
  auto mappoints = keyframe->mappoints();
  for (size_t i = 0; i < mappoints.size(); ++i) {
    auto mp = mappoints[i];
    if (mp && !mp->is_bad()) {
      if (!mp->IsObservedByKeyFrame(keyframe)) {
        mp->AddObservation(keyframe, i);
        mp->UpdateNormalAndDepth();
        mp->ComputeDistinctiveDescriptors();
      }
    }
  }

  // Associate the keyframe to already existing keyframes in the map
  keyframe->ConnectToMap();

  // Insert the keyframe into the map
  map_->AddKeyFrame(keyframe);
}

void Mapper::CullMapPoints() {
  
}

void Mapper::TriangulateNewMapPoints(std::shared_ptr<KeyFrame> keyframe) {
  // TUNE: 20
  auto neighbors_of_kf = keyframe->GetConnectedKeyFrames(20);
  cv::Mat o_w_kf = keyframe->o_w();
  // Match keyframe to its every neighbor with epipolar restriction
  // and triangulate new mappoints according to searched matches
  for (auto& neighbor : neighbors_of_kf) {
    // Check first that the baseline is not too short
    double baseline = cv::norm(neighbor->o_w() - o_w_kf);
    double scene_depth_neighbor = neighbor->SceneDepth(2);
    double ratio = baseline / scene_depth_neighbor;
    // TUNE: 0.01
    if (ratio < 0.01)
      continue;

   
    cv::Mat F = RecoverFundamental(keyframe, neighbor);

    // Search match
    auto matches = guided_matcher_->DbowAndEpipolarGuided2D2DMatcher(keyframe, neighbor, 50, false, false);
    
    // Triangulate each match

  }
}

} // namespace lslam