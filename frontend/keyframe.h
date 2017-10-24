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

#include "frame.h"

namespace lslam {

// TODO: Thread safe???

class KeyFrame : public Frame, public std::enable_shared_from_this<KeyFrame> {
public:
  // Constructor from base Frame object
  KeyFrame(const Frame& frame);

  // accessor
  unsigned long id() const;
  unsigned long frame_id() const;
  bool is_bad() const;
  std::set<std::shared_ptr<KeyFrame>> children_keyframes();
  std::shared_ptr<KeyFrame> parent_keyframe();
  
  // Connection functions
  void AddConnection(std::shared_ptr<KeyFrame> frame, const int weight);
  void ConnectToMap();
  void AddChild(std::shared_ptr<KeyFrame> child);
  std::vector<std::shared_ptr<KeyFrame>> GetConnectedKeyFrames(const size_t n = 0);

  // Compute Scene depth
  double SceneDepth(const int q);

private:
  // Unique keyframe id, 0,1,2,...
  unsigned long kf_id_; 
  // Connections to other keyframes
  // - Covisibility graph
  std::map<std::shared_ptr<KeyFrame>, int> connected_keyframes_weights_;
  std::vector<std::shared_ptr<KeyFrame>> sorted_connected_keyframes_; // sorted according to weights descending
  // Spanning tree
  bool is_new_nod_;
  std::shared_ptr<KeyFrame> parent_keyframe_;
  std::set<std::shared_ptr<KeyFrame>> children_keyframes_;

  // State
  bool is_bad_;
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
