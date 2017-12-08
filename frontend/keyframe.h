/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_KEYFRAME_H
#define FRONTEND_KEYFRAME_H

#include <memory>
#include <vector>
#include <map>
#include <set>
#include <atomic>
#include <mutex>

#include "frame.h"

namespace lslam {

class MapPoint;
class Map;

/**
 * @brief Thread-safe KeyFrame class.
 * 
 */
class KeyFrame : public Frame, public std::enable_shared_from_this<KeyFrame> {
public:
  // Constructor
  KeyFrame(const Frame& frame, const std::shared_ptr<Map>& map);
  // Forbid copy constructor 
  KeyFrame(const KeyFrame&) = delete;
  // Forbid assignment operator
  KeyFrame& operator=(const KeyFrame&) = delete;
  ~KeyFrame();

  // Accessors
  unsigned long id() const;
  unsigned long frame_id() const;

  // Thread-safe inherited Frame class accessors
  std::vector<std::shared_ptr<MapPoint>> mappoints() const;
  std::shared_ptr<MapPoint> mappoint(size_t idx) const; 
  // Pose
  cv::Mat T_cw() const;
  cv::Mat T_wc() const;
  cv::Mat o_w() const;
  cv::Mat R_cw() const;
  cv::Mat t_cw() const;

  // Thread-safe inherited Frame class mutators
  void SetPose(const cv::Mat& T_cw);
  void set_mappoint(size_t idx, std::shared_ptr<MapPoint> mappoint);
  void EraseMapPoint(const size_t& idx);
  void EraseMapPoint(std::shared_ptr<MapPoint> mappoint);

// Thread-safe accessors
  bool is_bad() const;
  std::set<std::shared_ptr<KeyFrame>> children_keyframes();
  std::shared_ptr<KeyFrame> parent_keyframe();
  
  // Connection functions
  void AddConnectedKeyFrame(std::shared_ptr<KeyFrame> frame, const int weight);
  void EraseConnectedKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  void SetConnectedKeyFrames();
  void AddChild(std::shared_ptr<KeyFrame> child);
  void EraseChild(std::shared_ptr<KeyFrame> child);
  std::vector<std::shared_ptr<KeyFrame>> GetConnectedKeyFrames(const size_t n = 0);
  int GetConnectedWeight(std::shared_ptr<KeyFrame> keyframe);
  void SetParent(std::shared_ptr<KeyFrame> parent);
  // Input: (x;y;z)
  double Depth(const cv::Mat& p3d);
  // Compute Scene depth
  double SceneDepth(const int q);

  void SetBadFlag();

public:
  struct compare {
    bool operator() (const std::shared_ptr<KeyFrame> lhs, const std::shared_ptr<KeyFrame> rhs) {
      return lhs->id() < rhs->id();
    }
  };

private:
  // Unique keyframe id, 0,1,2,...
  unsigned long kf_id_; 
  // Connections to other keyframes
  // - Covisibility graph
  std::vector<std::shared_ptr<KeyFrame>> sorted_connected_keyframes_; // sorted according to weights descending
  // Spanning tree
  bool is_new_nod_;
  std::shared_ptr<KeyFrame> parent_keyframe_;
  std::set<std::shared_ptr<KeyFrame>> children_keyframes_;

  // If is bad, the allocated memory will be deleted after the smart pointer's count decreases to 0.
  std::atomic<bool> is_bad_; ///< Indicate that this keyframe is invalid.
  std::shared_ptr<Map> map_;
  // Provide concurrency on pose matrix
  mutable std::mutex mutex_pos_; 
  // Provide concurrency on mappoints_, sorted_connected_keyframes_, connected_keyframes_weights_,
  // parent_keyframe_, children_keyframes_
  mutable std::mutex mutex_con_;
  // Provide concurrency on corresponding mappoints
  mutable std::mutex mutex_mps_;
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
