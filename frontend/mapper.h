/*
 * @Author: Shi Qin 
 * @Date: 2017-10-25 09:42:35 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 15:49:52
 */

#ifndef FRONTEND_MAPPING_H_
#define FRONTEND_MAPPING_H_

#include <memory>
#include <list>

namespace lslam {
  
class Map;
class KeyFrame;
class GuidedMatcher;
class MapPoint;

class Mapper {
public:
  Mapper() {}
  Mapper(std::shared_ptr<Map> map,
          std::shared_ptr<GuidedMatcher> guided_matcher,
          std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor);

  // Input: keyframe with initial pose and initial associated mappoints in map
  void Process(std::shared_ptr<KeyFrame> keyframe);

private:
  void InsertKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  void CullMapPoints();
  void TriangulateNewMapPoints(std::shared_ptr<KeyFrame> keyframe);
  void FuseAndAssociateMapPoints(std::shared_ptr<KeyFrame> keyframe);

private:
  std::shared_ptr<Map> map_;
  // Guided matcher
  std::shared_ptr<GuidedMatcher> guided_matcher_;
  
  std::list<std::shared_ptr<MapPoint>> triangulated_mappoints_;

  // Scale pyramid info.
  int max_level_;
  int scale_levels_;
  float scale_factor_;
  float log_scale_factor_;//
  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> level_sigma2_;
  std::vector<float> inv_level_sigma2_;
};

} // namespace lslam

#endif //FRONTEND_MAPPING_H_