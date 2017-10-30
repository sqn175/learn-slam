/*
 * @Author: Shi Qin 
 * @Date: 2017-10-25 09:42:35 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 15:49:52
 */

#ifndef FRONTEND_MAPPING_H_
#define FRONTEND_MAPPING_H_

#include <memory>

namespace lslam {
  
class Map;
class KeyFrame;
class GuidedMatcher;

class Mapper {
public:
  Mapper() {}
  Mapper(std::shared_ptr<Map> map,
          std::shared_ptr<GuidedMatcher> guided_matcher);

  void Process(std::shared_ptr<KeyFrame> keyframe);

private:
  void InsertKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  void CullMapPoints();
  void TriangulateNewMapPoints(std::shared_ptr<KeyFrame> keyframe);

private:
  std::shared_ptr<Map> map_;
  // Guided matcher
  std::shared_ptr<GuidedMatcher> guided_matcher_;
};

} // namespace lslam

#endif //FRONTEND_MAPPING_H_