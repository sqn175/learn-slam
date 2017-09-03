/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include <iostream>

#include "frontend.h"

#include "opencv2/core/eigen.hpp"

#include "helper.h"

namespace lslam {

Frontend::Frontend() : state_(FrontEndState::kNotReady) {
}

bool Frontend::Initialize() {
  
  // Find correspondences
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> matches;
  // KNN matcher, k = 2, find the best match and the second best match
  matcher.knnMatch(camera_measurement_prev_->descriptors(), camera_measurement_current_->descriptors(), matches, 2);
  
  std::vector<cv::DMatch> good_matches;
  
  // The ratio threshold, we may adjust this
  // Kick out the correspondences,
  // which distance ratio of best match and second best match is below the threshold
  const double ratio = 0.8;
  
  for ( auto it = matches.begin(); it != matches.end(); ++it){
    if ((*it)[0].distance < 50 && (*it)[0].distance < ratio * (*it)[1].distance)
      good_matches.push_back((*it)[0]);
  }
  
  size_t num_correspondences = good_matches.size();
  
  if (num_correspondences < 10) {
    // Too few correspondences
    return false;
  }
  
  // convert keypoint to vecctor<Point2f>
  std::vector<cv::Point2f> src_points, dst_points;
  for ( auto it = good_matches.begin(); it != good_matches.end(); ++it) {
    src_points.push_back( camera_measurement_prev_->keypoints()[(*it).queryIdx].pt );
    dst_points.push_back( camera_measurement_current_->keypoints()[(*it).trainIdx].pt );
  }
  
  // Find fundamental 
  cv::Mat K = camera_measurement_current_->camera_model()->K();
  cv::Mat fundamental_inliers_mask;
  cv::Mat fundamental_matrix = cv::findFundamentalMat(src_points, dst_points, cv::FM_RANSAC, 1.841, 0.99, fundamental_inliers_mask);
  cv::Mat essential_matrix = K.t() * fundamental_matrix * K;

  // Recover relative camera pose from essential matrix
  cv::Mat R, t;
  cv::recoverPose(essential_matrix, src_points, dst_points, K, R, t, cv::noArray());

  // Get quality
  int inlier_cnt = cv::countNonZero(fundamental_inliers_mask);
  double inlier_ratio = double(inlier_cnt) / double(num_correspondences);
  
  // Not enough inlier
  if (inlier_ratio < 0.8) 
    return false;
  
  // Result is good
  state_ = FrontEndState::kInitialized;
  
  // Initialize the previous camera ralative pose
  cv::Mat T_cw_prev = cv::Mat::eye(4,4,CV_64F);
  camera_measurement_prev_->SetPose(T_cw_prev);
  
  // Initialize the current camera relative pose
  cv::Mat T_cw_cur = cv::Mat::eye(4,4,CV_64F);
  R.copyTo(T_cw_cur.rowRange(0,3).colRange(0,3));
  t.copyTo(T_cw_cur.rowRange(0,3).col(3));
  camera_measurement_current_->SetPose(T_cw_cur);
  
  // Initial data association
  // We set the initial frame and current frame as keyframe
  auto keyframe_ini = std::make_shared<KeyFrame>(camera_measurement_prev_);
  auto keyframe_cur = std::make_shared<KeyFrame>(camera_measurement_current_);
  
  // Insert the two keyframes to map
  map_.AddKeyFrame(keyframe_ini);
  map_.AddKeyFrame(keyframe_cur);
  
  // Create landmarks and asscioate to keyframes
  // Triangulate keypoints to get landmarks
  cv::Mat world_coordinates;
  cv::triangulatePoints(K*T_cw_prev.rowRange(0,3), K*T_cw_cur.rowRange(0,3), src_points, dst_points, world_coordinates);
  for (size_t i = 0; i < src_points.size(); ++i ) {
    // TODO:: check landmark valid
    Eigen::Vector4d eigen_world_coor;
    cv::Mat x = world_coordinates.col((int)i);
    x /= x.at<float>(3,0);
    cv::cv2eigen(x, eigen_world_coor);
    if (eigen_world_coor[3] <= 0)
      continue;
    
    // Create landmark
    std::shared_ptr<lslam::Landmark> landmark = std::make_shared<lslam::Landmark> (eigen_world_coor);
    // Add 2 observation
    // The keyframe_ini is observating this landmark, the associated keypoint is queryIdx
    int query_idx = good_matches[i].queryIdx;
    landmark->AddObservation(keyframe_ini,query_idx);
    // The keyframe_cur is also observating this landmark
    int train_idx = good_matches[i].trainIdx;
    landmark->AddObservation(keyframe_cur,train_idx);
    
    // Update the best descriptor of this landmark
    landmark->ComputeDistinctiveDescriptors();
    // Add landmark to map
    map_.AddLandmarkPoint(landmark);
    
  }
  
  return true;
  
  
}

void Frontend::AddCameraMeasurement(std::shared_ptr<CameraMeasurement> camera_measurement_current) {
  
  camera_measurement_current_ = camera_measurement_current;
  
  // if initialized 
  if (state_ == FrontEndState::kInitialized) {
    // TRACK
    
  } else if (state_ == FrontEndState::kNotInitialized) {
    
    if (Initialize()) {
      state_ = FrontEndState::kInitialized;
      LOG(INFO) << "Initialized!";
    }
    camera_measurement_prev_ = camera_measurement_current_;
    
  } else if (state_ == FrontEndState::kNotReady) {
    // For monocular case, we need two frames for initialization
      camera_measurement_prev_ = camera_measurement_current_;
      state_ = FrontEndState::kNotInitialized;
      return;
  }
  
}

void Frontend::DataAssociation() {
  // If this is the first frame
  if ( map_.SizeOfKeyframe() == 0)
  {

  }
}

} // namespace lslam
