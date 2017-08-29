/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "map.h"

namespace lslam {

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  keyframes_.insert(keyframe);
}

void Map::AddLandmarkPoint(std::shared_ptr<LandmarkPoint> landmarkpoint) {
  landmarkpoints_.insert(landmarkpoint);
}

Map::SizeOfKeyframe() {
  return keyframes_.size();
}
} // namespace lslam
