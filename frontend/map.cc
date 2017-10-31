/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "map.h"
#include "mappoint.h"
#include "keyframe.h"
namespace lslam {

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_);
  keyframes_.insert(keyframe);
}

void Map::AddMapPoint(std::shared_ptr<MapPoint> mappoint) {
  std::unique_lock<std::mutex> lock(mutex_);
  mappoints_.insert(mappoint);
}

void Map::EraseMapPoint(std::shared_ptr<MapPoint> mappoint) {
  std::unique_lock<std::mutex> lock(mutex_);
  mappoints_.erase(mappoint);
  // TODO: check we delete the mappoint pointer and the memory
}

size_t Map::SizeOfKeyframes() {
  std::unique_lock<std::mutex> lock(mutex_);
  return keyframes_.size();
}

size_t Map::SizeOfMappoints() {
  std::unique_lock<std::mutex> lock(mutex_);
  return mappoints_.size();
}

std::vector<std::shared_ptr<KeyFrame>> Map::keyframes() {
  std::unique_lock<std::mutex> lock(mutex_);
  return std::vector<std::shared_ptr<KeyFrame>>(keyframes_.begin(),keyframes_.end());
}

std::vector<std::shared_ptr<MapPoint>> Map::mappoints(){
  std::unique_lock<std::mutex> lock(mutex_);
  return std::vector<std::shared_ptr<MapPoint>>(mappoints_.begin(),mappoints_.end());
}

} // namespace lslam
