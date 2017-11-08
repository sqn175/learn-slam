/*
 * Author: ShiQin
 * Date: 2017-08-25
 */
#include <iostream>
#include "keyframe.h"

#include <glog/logging.h>
#include "mappoint.h"

#include "../common/helper.h"

namespace lslam {

size_t DEBUG_KF_ID = 3;

KeyFrame::KeyFrame(const Frame& frame)
  : Frame(frame)
  , is_bad_(false)
  , is_new_nod_(true) {
  static unsigned long unique_id = 0;
  kf_id_ = unique_id++;

  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "was created.";
  }
}

KeyFrame::~KeyFrame() {
  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "was deleted.";
  }
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame> frame, const int weight) {
  connected_keyframes_weights_[frame] = weight;
  // sort
  sorted_connected_keyframes_.clear();
  auto sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);
  for(auto it = sorted_connected_keyframes_weights_.rbegin(); it != sorted_connected_keyframes_weights_.rend(); ++it) {
    sorted_connected_keyframes_.push_back(it->second);
  }
}

void KeyFrame::ConnectToMap() {
  if (kf_id_==5) {
    int test = 0;
    int test1 = test;
  }

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
      // set parent, TODO: use the addparent interface
      parent_keyframe_ = parent_keyframe;
      parent_keyframe_->AddChild(shared_from_this());
      is_new_nod_ = false;
    }
  }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame> child) {
  children_keyframes_.insert(child);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame> child) {
  children_keyframes_.erase(child);
}

int KeyFrame::GetConnectedWeight(std::shared_ptr<KeyFrame> keyframe) {
  if (connected_keyframes_weights_.count(keyframe)) 
    return connected_keyframes_weights_[keyframe];
  else
    return 0;
}

void KeyFrame::SetParent(std::shared_ptr<KeyFrame> parent) {
  parent_keyframe_ = parent;
  parent_keyframe_->AddChild(shared_from_this());
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
  cv::Mat R_wc2 = R_cw_.row(2).t();
  return R_wc2.dot(p3d) + t_cw_.at<double>(2);
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

void KeyFrame::EraseMapPoint(std::shared_ptr<MapPoint> mappoint) {
  int idx = mappoint->GetIndexInKeyFrame(shared_from_this());
  // test
  if (idx == 1685)
    std::cout << "mappoint:"<<mappoint->id()<<" erased."<<std::endl;
  if (idx >= 0)
    mappoints_[idx].reset();
}

void KeyFrame::SetBadFlag() {
  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "was set as bad.";
  }
  for (auto& kf : sorted_connected_keyframes_) {
    kf->EraseConnectedKeyFrame(shared_from_this());
  }

  for (auto& mp : mappoints_)
    if (mp) // TODO: check if only two observations left and set this mp bad 
      mp->EraseObservation(shared_from_this());

  connected_keyframes_weights_.clear();
  sorted_connected_keyframes_.clear();

  // Update Spanning Tree
  std::set<std::shared_ptr<KeyFrame>> parent_candidates;
  parent_candidates.insert(parent_keyframe_);

  while (!children_keyframes_.empty()) {
    bool con = false;
    int max = -1;
    std::shared_ptr<KeyFrame> child;
    std::shared_ptr<KeyFrame> parent;

    for (auto& child_kf : children_keyframes_) {
      if (child_kf->is_bad())
        continue;

      // Check if a parent candidate is connected to the child keyframe 
      // Or to the connected keyframes of the child keyframe
      // ORB_SLAM2 only check the first rule
      auto connected_kfs = child_kf->GetConnectedKeyFrames();
      for (auto& connected_kf : connected_kfs) {
        for (auto& parent_candidate : parent_candidates) {
          // TODO: consider if we should check (connected_kf == parent_candidate)
          // TODO: if we should check (connected_kf->id() < child_kf->id()), rewrite this
          if (connected_kf->id() == parent_candidate->id()) {
            int w = child_kf->GetConnectedWeight(connected_kf);
            if (w>max) {
              child = child_kf;
              parent = connected_kf;
              max = w;
              con = true;
            }
          }
        }
      }
    }
    
    if (con) {
      child->SetParent(parent);
      parent_candidates.insert(child);
      children_keyframes_.erase(child);
    } else {
      break;
    }
  }

  // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
  if (!children_keyframes_.empty()) {
    for (auto& child_kf : children_keyframes_) 
      child_kf->SetParent(parent_keyframe_);
  }

  // Update parent 
  parent_keyframe_->EraseChild(shared_from_this());

  is_bad_ = true;
}

void KeyFrame::EraseConnectedKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  if (connected_keyframes_weights_.count(keyframe)) {
    connected_keyframes_weights_.erase(keyframe);
    // sort TODO: wrap this as a function to be called after modification of connected_keyframes_weights_
    sorted_connected_keyframes_.clear();
    auto sorted_connected_keyframes_weights_ = FlipMap(connected_keyframes_weights_);
    for(auto it = sorted_connected_keyframes_weights_.rbegin(); it != sorted_connected_keyframes_weights_.rend(); ++it) {
      sorted_connected_keyframes_.push_back(it->second);
    }
  }
}

} // namespace lslam
