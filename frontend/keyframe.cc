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
  sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);
}

void KeyFrame::UpdateConnections() {
  std::map<std::shared_ptr<KeyFrame>, int> all_connected_keyframes_weights;
  connected_keyframes_weights_.clear();
  sorted_connected_keyframes_weights_.clear();
  // Iterate the mappoints associated to this keyframe, check in which other keyframes are they seen
  for (auto& mp : mappoints_) {
    if (!mp)
      continue;
    
    std::map<std::shared_ptr<KeyFrame>, size_t> observations = mp->observations();
    for (auto& ob : observations) {
      if (ob.first->id() == id_)
        continue;
        all_connected_keyframes_weights[ob.first]++;
    }
  }

  // If the weight is greater than threshold, we add connection
  // In case no keyframe counter is over threshold add the one with maximum weight
  int max_weight = 0;
  std::shared_ptr<KeyFrame> max_weight_keyframe;
  // TUNE: 15
  int th = 15;

  for (auto& item : all_connected_keyframes_weights) {
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
  sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);

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

double KeyFrame::SceneDepth(const int q){
  std::vector<double> depths;
  for (auto& mp : mappoints_) {
    if (mp) {
      cv::Mat image_3d = Project(mp->pt_world());
      depths.push_back(image_3d.at<double>(2));
    }
  }
  sort(depths.begin(), depths.end());
  return depths[(depths.size()-1)/q];
}


} // namespace lslam
