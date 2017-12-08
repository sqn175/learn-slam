/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "frame.h"

#include <glog/logging.h>
#include "../3rdparty/ORB_SLAM2_modified/Converter.h"

#include <iostream>
#include "my_assert.h"
#include "mappoint.h"
#include "keyframe.h"

namespace lslam {

extern int DEBUG_ID;
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
  
Frame::Frame() {
}

Frame::Frame(const cv::Mat& image,double timestamp, 
             std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor,
             std::shared_ptr<PinholeCamera> camera_model,
             std::shared_ptr<ORBVocabulary> orb_voc)
  : timestamp_(timestamp)
  , orb_extractor_(orb_extractor)
  , camera_model_(camera_model)
  , orb_voc_(orb_voc)
{
  static unsigned long unique_id = 0;
  id_ = unique_id++;
  
  // Extract ORB
  CHECK(image.data) << "This Frame is not initialized with image";
  (*orb_extractor_)(image, cv::Mat(), keypoints_, descriptors_);
  
  // Undistort
  if (camera_model->DistortionType().compare("radialtangential") == 0 ) {
    cv::Mat mat(keypoints_.size(), 2, CV_32F);
    for (int i = 0 ; i < keypoints_.size(); ++i) {
      mat.at<float>(i,0) = keypoints_[i].pt.x;
      mat.at<float>(i,1) = keypoints_[i].pt.y;
    }
    // Undistort keypoints using OpenCV function
    mat = mat.reshape(2);

    cv::undistortPoints(mat, mat, camera_model->K(), camera_model->DistCoeffs(), cv::noArray(), camera_model->K());
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    undistorted_kps_.resize(keypoints_.size());
    for (int i = 0; i < keypoints_.size(); ++i) {
      cv::KeyPoint kp = keypoints_[i];
      kp.pt.x = mat.at<float>(i,0);
      kp.pt.y = mat.at<float>(i,1);
      undistorted_kps_[i] = kp;
    }

  } else {
    undistorted_kps_ = keypoints_;
  }

  // SetPose
  SetPose(cv::Mat::eye(4,4,CV_64F));

  // Allocate mappoints of NULL
  mappoints_ = std::vector<std::shared_ptr<MapPoint>>(keypoints_.size(), static_cast<std::shared_ptr<MapPoint>>(NULL));

  // Initialize outlier flag
  outliers_ = std::vector<bool>(mappoints_.size(), false);

  // Build gridding range searcher TODO: consider if we need to build grid at the first time
  range_searcher_ = std::make_shared<RangeSearcher>(undistorted_kps_, camera_model->image_bounds(), 1);
}

Frame::Frame(const Frame& frame) 
  : timestamp_(frame.timestamp_), id_(frame.id_)
  , camera_model_(frame.camera_model_), orb_extractor_(frame.orb_extractor_), orb_voc_(frame.orb_voc_)
  , keypoints_(frame.keypoints_), undistorted_kps_(frame.undistorted_kps_), descriptors_(frame.descriptors_.clone())
  , mappoints_(frame.mappoints_), outliers_(frame.outliers_)
  , T_cw_(frame.T_cw_.clone()), R_cw_(frame.R_cw_.clone()), t_cw_(frame.t_cw_.clone()), o_w_(frame.o_w_.clone())
  , T_wc_(frame.T_wc_.clone())
  , connected_keyframes_weights_(frame.connected_keyframes_weights_)
  , local_mappoints_(frame.local_mappoints_)
  , range_searcher_(frame.range_searcher_) {
}

unsigned long Frame::id() const {
  return id_;
}

std::vector<cv::KeyPoint> Frame::keypoints() const {
  return keypoints_;
}

const cv::KeyPoint& Frame::keypoint(size_t idx) const {
  return keypoints_[idx];
}

const std::vector<cv::KeyPoint>& Frame::undistorted_kps() const {
  return undistorted_kps_;
}

const cv::Mat& Frame::descriptors() const {
  return descriptors_;
}

const cv::KeyPoint& Frame::undistorted_kp(size_t idx) const {
  return undistorted_kps_[idx];
}

std::vector<std::shared_ptr<MapPoint>> Frame::mappoints() const {
  return mappoints_;
}

std::shared_ptr<RangeSearcher> Frame::range_searcher() const {
  return range_searcher_;
}

std::shared_ptr<PinholeCamera> Frame::camera_model() const {
  return camera_model_;
}

std::shared_ptr<ORB_SLAM2::ORBextractor> Frame::orb_extractor() const {
  return orb_extractor_;
}

bool Frame::outlier(size_t idx) const {
  return outliers_[idx];
}

const DBoW2::BowVector& Frame::bow_vector() const {
  return bow_vector_;
}
const DBoW2::FeatureVector& Frame::feature_vector() const {
  return feature_vector_;
}

std::set<std::shared_ptr<MapPoint>> Frame::connected_mappoints() const {
  return local_mappoints_;
}

/**
 * @brief Set pose matrix of the frame.
 * 
 * @param T_cw Transformation matrix from world coordinate to camera coordinate
 */
void Frame::SetPose(const cv::Mat& T_cw) {
  T_cw_ = T_cw.clone();
  R_cw_ = T_cw.rowRange(0,3).colRange(0,3);
  t_cw_ = T_cw.rowRange(0,3).col(3);
  o_w_ = -R_cw_.t() * t_cw_;
  
  T_wc_ = cv::Mat::eye(4,4,T_cw_.type());
  cv::Mat R_wc_ = R_cw_.t();
  R_wc_.copyTo(T_wc_.rowRange(0,3).colRange(0,3));
  o_w_.copyTo(T_wc_.rowRange(0,3).col(3));
}

void Frame::set_outlier(size_t idx, bool flag) {
  outliers_[idx] = flag;
}


void Frame::set_mappoint( size_t idx, std::shared_ptr<MapPoint> mp) {
  mappoints_[idx] = mp;
  LOG_IF(INFO,mp&& mp->id()==DEBUG_ID) << "Mappoint " << DEBUG_ID << " was mathced to frame id = " << id_ << " at idx = " <<idx;
}

std::shared_ptr<MapPoint> Frame::mappoint(size_t idx) const {
  return mappoints_[idx];
}

// TODO: consider if we need deep clone?
cv::Mat Frame::T_cw() const {
  return T_cw_.clone();
}

cv::Mat Frame::T_wc() const {
  return T_wc_.clone();
}

cv::Mat Frame::o_w() const {
  return o_w_.clone();
}

cv::Mat Frame::R_cw() const {
  return R_cw_.clone();
}
cv::Mat Frame::t_cw() const {
  return t_cw_.clone();
}

cv::Mat Frame::Project(const cv::Mat pt3d_w) {
  CHECK(pt3d_w.cols == 1 && pt3d_w.rows == 3) << "Invalid 3d point.";
  CHECK(T_cw_.data) << "The pose of frame is NOT set.";
  cv::Mat p_c = R_cw_*pt3d_w + t_cw_;
  return p_c;
}

cv::Mat Frame::KRtProject(const cv::Mat& p3d) {
  CHECK(camera_model_);
  cv::Mat p_c = Project(p3d);
  return camera_model_->Project(p_c);
}

void Frame::ComputeBoW() {
  if (bow_vector_.empty() || feature_vector_.empty()) {
    CHECK(descriptors_.data);
    std::vector<cv::Mat> desc_mat = ORB_SLAM2::Converter::toDescriptorVector(descriptors_);
    orb_voc_->transform(desc_mat, bow_vector_, feature_vector_, 4);
  }
}

void Frame::SetConnectedKeyFrames() {
  // Iterate the mappoints associated to this keyframe, check in which other keyframes are they seen
  connected_keyframes_weights_.clear();

  for(size_t i = 0; i < mappoints_.size(); ++i) {
    auto mp = mappoints_[i];
    if (!mp)
      continue;
  
    // TODO: why I need to check is_bad  
    if (!mp->is_bad()) {
      auto observations = mp->observations();
      for (auto& ob : observations) {
        // This observation keyframe is created from this frame
        if (ob.first->frame_id() == id_ )
          continue;
        connected_keyframes_weights_[ob.first]++;
      }
    } else {
      EraseMapPoint(i);
    }
  }

  // TODO: remove this part out, as we dont need to store the connected_mappoints int the Frame class.
  std::set<std::shared_ptr<KeyFrame>> all_connected_keyframes;
  // Consider the neighbors of these direct connected keyframes
  for (auto& kf_w : connected_keyframes_weights_) {
    auto kf = kf_w.first;
    if (kf->is_bad())
      continue;

    all_connected_keyframes.insert(kf);
    
    // TUNE: 10
    auto neighbors_of_kf = kf->GetConnectedKeyFrames(10);
    for (auto& neighbor : neighbors_of_kf) {
      if (neighbor->is_bad() || neighbor->frame_id() == id_)
        continue;

      all_connected_keyframes.insert(neighbor);
      // TODO: ORB_SLAM2 have a break here, it only accept the best first one, the following loops is the same
    }

    auto children_of_kf = kf->children_keyframes();
    for(auto& child : children_of_kf) {
      if (child->is_bad() || child->frame_id() == id_)
        continue;

      all_connected_keyframes.insert(child);
    }

    auto parent_of_kf = kf->parent_keyframe();
    if (parent_of_kf && !(parent_of_kf->is_bad() || parent_of_kf->frame_id() == id_)) {
      all_connected_keyframes.insert(parent_of_kf);
    }
  }

  // Retrive all mappoints from all connected keyframes
  local_mappoints_.clear();
  for(auto& kf : all_connected_keyframes) {
    auto mps_of_kf = kf->mappoints();
    for(auto& mp : mps_of_kf) {
      if (!mp || mp->is_bad()) 
        continue;

      local_mappoints_.insert(mp);
     }
  }
  // We erase the mappoints already associated to keypoints 
  // in the previous TrackToLastFrame or TrackToLastKeyFrame functions
  for (auto& mp:mappoints_) {
    if (mp) 
      local_mappoints_.erase(mp);
  }
}

void Frame::EraseMapPoint(const size_t& idx) {
  mappoints_[idx].reset();
}

} // namespace lslam
