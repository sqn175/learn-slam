/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_MAP_H
#define FRONTEND_MAP_H

#include <memory>
#include <set>
#include <vector>
#include <mutex>

namespace lslam {

class MapPoint;
class KeyFrame;

// Thread safe map
class Map {
public:
  Map() { }
  ~Map() { }
  
  void AddKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  void AddMapPoint(std::shared_ptr<MapPoint> mappoint);
  
  size_t SizeOfKeyframes();
  size_t SizeOfMappoints();
  
  std::vector<std::shared_ptr<KeyFrame>> keyframes();
  std::vector<std::shared_ptr<MapPoint>> mappoints();
  
private:
  std::set<std::shared_ptr<KeyFrame>> keyframes_; // all keyframes in the map
  // TODO: consider do we need mappoints_ as member
  std::set<std::shared_ptr<MapPoint>> mappoints_; // all mappoints in the map

  std::mutex mutex_;
};

} // namespace lslam

#endif // FRONTEND_MAP_H
