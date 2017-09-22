/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include <iostream>
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

std::vector<std::shared_ptr<KeyFrame>> Map::keyframes() const {
  return std::vector<std::shared_ptr<KeyFrame>>(keyframes_.begin(),keyframes_.end());
}

std::vector<std::shared_ptr<Landmark>> Map::landmarkpoints() const {
  std::vector<std::shared_ptr<Landmark>> v(landmarkpoints_.begin(),landmarkpoints_.end());
  return v;
}

} // namespace lslam
