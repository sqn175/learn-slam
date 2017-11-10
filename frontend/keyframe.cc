/*
 * Author: ShiQin
 * Date: 2017-08-25
 */
#include <iostream>
#include <string>
#include "keyframe.h"

#include <glog/logging.h>
#include "mappoint.h"

#include "../common/helper.h"
#include "map.h"

namespace lslam {

int DEBUG_KF_ID = -1;

/**
 * @brief Construct a keyframe from a frame, and insert the keyframe to map.
 *        A deep copy is used, so the keyframe bear no relation to the frame. 
 * 
 * @param frame Frame which is set to be a keyframe.
 */
KeyFrame::KeyFrame(const Frame& frame, const std::shared_ptr<Map>& map)
  : Frame(frame)
  , is_bad_(false)
  , is_new_nod_(true)
  , map_(map) {
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
    DLOG(INFO) << "Keyframe " << DEBUG_KF_ID << "was deleted.";
  }
}

std::vector<std::shared_ptr<MapPoint>> KeyFrame::mappoints() const {
  std::unique_lock<std::mutex> lock(mutex_mps_);
  return Frame::mappoints();
}

std::shared_ptr<MapPoint> KeyFrame::mappoint(size_t idx) const {
  std::unique_lock<std::mutex> lock(mutex_mps_);
  return Frame::mappoint(idx);
}

cv::Mat KeyFrame::T_cw() const {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  return Frame::T_cw();
}
cv::Mat KeyFrame::T_wc() const {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  return Frame::T_wc();
}
cv::Mat KeyFrame::o_w() const {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  return Frame::o_w();
}
cv::Mat KeyFrame::R_cw() const {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  return Frame::R_cw();
}
cv::Mat KeyFrame::t_cw() const {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  return Frame::t_cw();
}

/**
 * @brief Set pose matrix of the keyframe.
 * 
 * @param T_cw Transformation matrix from world coordinate to camera coordinate
 */
void KeyFrame::SetPose(const cv::Mat& T_cw) {
  std::unique_lock<std::mutex> lock(mutex_pos_);
  Frame::SetPose(T_cw);
}

void KeyFrame::set_mappoint(size_t idx, std::shared_ptr<MapPoint> mappoint) {
  std::unique_lock<std::mutex> lock(mutex_mps_);
  Frame::set_mappoint(idx, mappoint);
}

void KeyFrame::EraseMapPoint(const size_t& idx) {
  std::unique_lock<std::mutex> lock(mutex_mps_);
  Frame::EraseMapPoint(idx);
}

/**
 * @brief Add a keyframe connection to this keyframe and sort all connections.
 * 
 * @param frame The connected keyframe.
 * @param weight Number of mappoints the connected keyframes co-observed.
 */
void KeyFrame::AddConnectedKeyFrame(std::shared_ptr<KeyFrame> frame, const int weight) {
  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "add a connection {kf_id, weight}: {" 
              << frame->id() << ", " << weight << "}.";
  }

  std::unique_lock<std::mutex> lock(mutex_con_);
  // We insert a new connection, if exists, we update it.
  connected_keyframes_weights_[frame] = weight;

  // Sort connected keyframes according to weight, update sorted_connected_keyframes_
  sorted_connected_keyframes_.clear();
  auto sorted_connected_weights_keyframes = FlipMap(connected_keyframes_weights_);
  for(auto it = sorted_connected_weights_keyframes.rbegin(); it != sorted_connected_weights_keyframes.rend(); ++it) {
    sorted_connected_keyframes_.push_back(it->second);
  }
}

/**
 * @brief Retrive connected keyframes which co-observe the same mappoints.
 *        In this way, the keyframe become a node of the covisibility graph and the spanning tree.
 * 
 */
void KeyFrame::SetConnectedKeyFrames() {

  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "start to setup connections.";
  }
  
  std::map<std::shared_ptr<KeyFrame>, int> connected_kf_w;
  
  // Thread-safe copy
  auto mps = mappoints();

  for(size_t i = 0; i < mps.size(); ++i) {
    auto mp = mps[i];
    if (!mp)
      continue;
  
    // TODO: why I need to check is_bad  
    if (!mp->is_bad()) {
      auto observations = mp->observations();
      for (auto& ob : observations) {
        // This observation keyframe is created from this frame
        if (ob.first->frame_id() == id_ )
          continue;
          connected_kf_w[ob.first]++;
      }
    } else {
      EraseMapPoint(i);
    }
  }

  // If the weight is greater than threshold, we add connection
  // In case no keyframe counter is beyond threshold add the one with maximum weight
  int max_weight = 0;
  std::shared_ptr<KeyFrame> max_weight_keyframe;
  // TUNE: 15
  int th = 15;
  
  for (auto it = connected_kf_w.begin(); it != connected_kf_w.end(); ) {
    if (it->second > max_weight) {
      max_weight = it->second;
      max_weight_keyframe = it->first;
    }
    if (it->second <= th) {
      it = connected_kf_w.erase(it);
    } else {
      // Add a connection between keyframes
      it->first->AddConnectedKeyFrame(shared_from_this(), it->second);
      ++it;
    }
  }

  if (connected_kf_w.empty()) {
    connected_kf_w[max_weight_keyframe] = max_weight;
    max_weight_keyframe->AddConnectedKeyFrame(shared_from_this(), max_weight);
  }

  // sort
  auto sorted_connected_weights_keyframes = FlipMap(connected_kf_w);

  // Thread-safe setting
  std::unique_lock<std::mutex> lock(mutex_con_);

  connected_keyframes_weights_ = std::move(connected_kf_w);

  sorted_connected_keyframes_.clear();
  for(auto it = sorted_connected_weights_keyframes.rbegin(); it != sorted_connected_weights_keyframes.rend(); ++it) {
    sorted_connected_keyframes_.push_back(it->second);
  }

  // Spanning tree
  if (is_new_nod_ && sorted_connected_weights_keyframes.rbegin() != sorted_connected_weights_keyframes.rend()) {
    std::shared_ptr<KeyFrame> parent_keyframe = sorted_connected_weights_keyframes.rbegin()->second;
    if (parent_keyframe->id() < id_) {
      parent_keyframe_ = parent_keyframe;
      parent_keyframe_->AddChild(shared_from_this());
      is_new_nod_ = false;
    }
  }

  // test
  if (kf_id_ == DEBUG_KF_ID) {
    auto test_kfs = sorted_connected_keyframes_;
    std::string test_info;
    test_info = "Keyframe " + std::to_string(DEBUG_KF_ID) + "initially set connections :";
    for (auto& kf : test_kfs) {
      test_info += std::to_string(kf->id()) + "  ";
    }
    test_info += "\n parent keyframe: " + std::to_string(parent_keyframe_->id()) + "\n";
    test_info += "children keyframes: ";
    for (auto& child : children_keyframes_) {
      test_info += std::to_string(child->id()) + "  ";
    }
    LOG(INFO) << test_info;            
  }
}

/**
 * @brief Add a child keyframe.
 * 
 * @param child Child keyframe.
 */
void KeyFrame::AddChild(std::shared_ptr<KeyFrame> child) {
  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "add a child: "<<child->id();
  }
  std::unique_lock<std::mutex> lock(mutex_con_);
  children_keyframes_.insert(child);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame> child) {
 // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "erased a child: "<<child->id();
  }

  std::unique_lock<std::mutex> lock(mutex_con_);
  children_keyframes_.erase(child);
}

int KeyFrame::GetConnectedWeight(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_con_);
  if (connected_keyframes_weights_.count(keyframe)) 
    return connected_keyframes_weights_[keyframe];
  else
    return 0;
}

void KeyFrame::SetParent(std::shared_ptr<KeyFrame> parent) {
   // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "set parent : "<<parent->id();
  }

  std::unique_lock<std::mutex> lock(mutex_con_);
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
  std::unique_lock<std::mutex> lock(mutex_con_);
  return children_keyframes_;
}

std::shared_ptr<KeyFrame> KeyFrame::parent_keyframe() {
  std::unique_lock<std::mutex> lock(mutex_con_);
  return parent_keyframe_;
}
std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames(const size_t n) {
  std::unique_lock<std::mutex> lock(mutex_con_);
  if ( n == 0 || n > sorted_connected_keyframes_.size()) {
    return sorted_connected_keyframes_;
  } else {
    return std::vector<std::shared_ptr<KeyFrame>>(sorted_connected_keyframes_.begin(), sorted_connected_keyframes_.begin()+n);
  }
}

double KeyFrame::Depth(const cv::Mat& p3d) {
  CHECK(p3d.data && p3d.rows == 3 && p3d.cols == 1);
  cv::Mat R = R_cw();
  cv::Mat t = t_cw();
  cv::Mat R_wc2 = R.row(2).t();
  return R_wc2.dot(p3d) + t.at<double>(2);
}

double KeyFrame::SceneDepth(const int q){
  std::vector<double> depths;
  auto mps = mappoints();
  for (auto& mp : mps) {
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
  if (idx >= 0) {
    std::unique_lock<std::mutex> lock(mutex_mps_);
    Frame::EraseMapPoint(idx);
  }
}

void KeyFrame::SetBadFlag() {
  // test
  if (kf_id_ == DEBUG_KF_ID) {
    LOG(INFO) << "Keyframe " << DEBUG_KF_ID << "was set as bad.";
  }
  // Set bad flag
  is_bad_ = true;

  std::unique_lock<std::mutex> lock1(mutex_mps_);
  for (auto& mp : mappoints_)
  if (mp) // TODO: check if only two observations left and set this mp bad 
    mp->EraseObservation(shared_from_this());
  lock1.unlock();

  std::unique_lock<std::mutex> lock2(mutex_con_);
  for (auto& kf : sorted_connected_keyframes_) {
    kf->EraseConnectedKeyFrame(shared_from_this());
  }

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

  // Erase from map
  map_->EraseKeyFrame(shared_from_this());
}

void KeyFrame::EraseConnectedKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_con_);
  if (connected_keyframes_weights_.count(keyframe)) {
    connected_keyframes_weights_.erase(keyframe);

    // sort TODO: wrap this as a function to be called after modification of connected_keyframes_weights_
    sorted_connected_keyframes_.clear();
    auto sorted_connected_weights_keyframes = FlipMap(connected_keyframes_weights_);
    for(auto it = sorted_connected_weights_keyframes.rbegin(); it != sorted_connected_weights_keyframes.rend(); ++it) {
      sorted_connected_keyframes_.push_back(it->second);
    }
  }
}

} // namespace lslam
