/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "map.h"

namespace lslam {

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_);
  keyframes_.insert(keyframe);
}

void Map::AddLandmarkPoint(std::shared_ptr<Landmark> landmarkpoint) {
  std::unique_lock<std::mutex> lock(mutex_);
  landmarkpoints_.insert(landmarkpoint);
}

size_t Map::SizeOfKeyframe() {
  std::unique_lock<std::mutex> lock(mutex_);
  return keyframes_.size();
}

std::vector<std::shared_ptr<KeyFrame>> Map::keyframes() {
  std::unique_lock<std::mutex> lock(mutex_);
  return std::vector<std::shared_ptr<KeyFrame>>(keyframes_.begin(),keyframes_.end());
}

std::vector<std::shared_ptr<Landmark>> Map::landmarkpoints(){
  std::unique_lock<std::mutex> lock(mutex_);
  return std::vector<std::shared_ptr<Landmark>>(landmarkpoints_.begin(),landmarkpoints_.end());
}

} // namespace lslam
