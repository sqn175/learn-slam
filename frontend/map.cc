/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "map.h"

namespace lslam {

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  keyframes_.insert(keyframe);
}

void Map::AddLandmarkPoint(std::shared_ptr<Landmark> landmarkpoint) {
  landmarkpoints_.insert(landmarkpoint);
}

size_t Map::SizeOfKeyframe() const {
  return keyframes_.size();
}
} // namespace lslam
