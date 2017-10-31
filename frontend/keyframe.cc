/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#include "keyframe.h"
#include "mappoint.h"

#include "../common/helper.h"

namespace lslam {

KeyFrame::KeyFrame(const Frame& frame)
  : Frame(frame)
  , is_bad_(false) {
  static unsigned long unique_id = 0;
  kf_id_ = unique_id++;
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame> frame, const int weight) {
  connected_keyframes_weights_[frame] = weight;
  // sort
  // sort
  auto sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);
  for(auto it = sorted_connected_keyframes_weights_.rbegin(); it != sorted_connected_keyframes_weights_.rend(); ++it) {
    sorted_connected_keyframes_.push_back(it->second);
  }
}

void KeyFrame::ConnectToMap() {
  connected_keyframes_weights_.clear();
  sorted_connected_keyframes_.clear();
  
  Frame::ConnectToMap();

  // If the weight is greater than threshold, we add connection
  // In case no keyframe counter is over threshold add the one with maximum weight
  int max_weight = 0;
  std::shared_ptr<KeyFrame> max_weight_keyframe;
  // TUNE: 15
  int th = 15;

  for (auto& item : direct_connected_keyframes_weights_) {
    if (item.second > max_weight) {
      max_weight = item.second;
      max_weight_keyframe = item.first;
    }
    if (item.second > th) {
      connected_keyframes_weights_[item.first] = item.second;
      item.first->AddConnection(shared_from_this(), item.second);
    }
  }

  if (connected_keyframes_weights_.empty()) {
    connected_keyframes_weights_[max_weight_keyframe] = max_weight;
    max_weight_keyframe->AddConnection(shared_from_this(), max_weight);
  }

  // sort
  auto sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);
  for(auto it = sorted_connected_keyframes_weights_.rbegin(); it != sorted_connected_keyframes_weights_.rend(); ++it) {
    sorted_connected_keyframes_.push_back(it->second);
  }

  // Spanning tree
  if (is_new_nod_ && sorted_connected_keyframes_weights_.rbegin() != sorted_connected_keyframes_weights_.rbegin()) {
    std::shared_ptr<KeyFrame> parent_keyframe = sorted_connected_keyframes_weights_.rbegin()->second;
    if (parent_keyframe->id() < id_) {
      parent_keyframe_ = parent_keyframe;
      parent_keyframe_->AddChild(shared_from_this());
      is_new_nod_ = false;
    }
  }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame> child) {
  children_keyframes_.insert(child);
}

unsigned long KeyFrame::id() const {
  return kf_id_;
}

unsigned long KeyFrame::frame_id() const {
  return id_;
}

bool KeyFrame::is_bad() const {
  return is_bad_;
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::children_keyframes() {
  return children_keyframes_;
}

std::shared_ptr<KeyFrame> KeyFrame::parent_keyframe() {
  return parent_keyframe_;
}
std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames(const size_t n) {
  if ( n == 0 || n > sorted_connected_keyframes_.size()) {
    return sorted_connected_keyframes_;
  } else {
    return std::vector<std::shared_ptr<KeyFrame>>(sorted_connected_keyframes_.begin(), sorted_connected_keyframes_.begin()+n);
  }
}

double KeyFrame::Depth(const cv::Mat& p3d) {
  CHECK(p3d.data && p3d.rows == 3 && p3d.cols == 1);
  return R_cw_.row(2).dot(p3d) + t_cw_.at<double>(2);
}

double KeyFrame::SceneDepth(const int q){
  std::vector<double> depths;
  for (auto& mp : mappoints_) {
    if (mp) {
      // TODO: using frame::depth()
      double depth = Depth(mp->pt_world());
      depths.push_back(depth);
    }
  }
  sort(depths.begin(), depths.end());
  return depths[(depths.size()-1)/q];
}


} // namespace lslam
