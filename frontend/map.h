/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_MAP_H
#define FRONTEND_MAP_H

#include <memory>
#include <set>
#include <mutex>

#include "keyframe.h"
#include "landmark.h"

namespace lslam {

// Thread safe map
class Map {
public:
  Map() { }
  ~Map() { }
  
  void AddKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  void AddLandmarkPoint(std::shared_ptr<Landmark> landmarkpoint);
  
  size_t SizeOfKeyframe();
  
  std::vector<std::shared_ptr<KeyFrame>> keyframes();
  std::vector<std::shared_ptr<Landmark>> landmarkpoints();
  
private:
  std::set<std::shared_ptr<KeyFrame>> keyframes_; // all keyframes in the map
  std::set<std::shared_ptr<Landmark>> landmarkpoints_; // all landmark points in the map

  std::mutex mutex_;
};

} // namespace lslam

#endif // FRONTEND_MAP_H
