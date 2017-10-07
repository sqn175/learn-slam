/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include <iostream>
#include <fstream>

#include "frontend.h"

#include "opencv2/core/eigen.hpp"

#include "helper.h"

namespace lslam {

Frontend::Frontend() : state_(FrontEndState::kNotReady) {
}

bool Frontend::DataAssociationBootstrap() {
  
  // Find correspondences
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> matches;
  // KNN matcher, k = 2, find the best match and the second best match
  matcher.knnMatch(last_frame_->descriptors(), cur_frame_->descriptors(), matches, 2);
  
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
    src_points.push_back( last_frame_->keypoints()[(*it).queryIdx].pt );
    dst_points.push_back( cur_frame_->keypoints()[(*it).trainIdx].pt );
  }
  
  // Find fundamental 
  cv::Mat K = camera_model_->K();
  cv::Mat fundamental_inliers_mask;
  cv::Mat fundamental_matrix = cv::findFundamentalMat(src_points, dst_points, cv::FM_RANSAC, 1.841, 0.99, fundamental_inliers_mask);
  cv::Mat essential_matrix = K.t() * fundamental_matrix * K;

  // Recover relative camera pose from essential matrix
  cv::Mat R, t;
  cv::recoverPose(essential_matrix, src_points, dst_points, K, R, t, cv::noArray());

  // test
  std::cout<<R<<std::endl;
  std::cout<<t<<std::endl;
  
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
  last_frame_->SetPose(T_cw_prev);
  
  // Initialize the current camera relative pose
  cv::Mat T_cw_cur = cv::Mat::eye(4,4,CV_64F);
  R.copyTo(T_cw_cur.rowRange(0,3).colRange(0,3));
  t.copyTo(T_cw_cur.rowRange(0,3).col(3));
  cur_frame_->SetPose(T_cw_cur);
  cur_frame_->set_T_cl(T_cw_cur*last_frame_->T_wc());

  std::cout<<"T_cw_prev"<<T_cw_prev<<std::endl;
  std::cout<<"T_cw_cur"<<T_cw_cur<<std::endl;
  
  // Initial data association
  // We set the initial frame and current frame as keyframe
  auto keyframe_ini = std::make_shared<KeyFrame>(last_frame_);
  auto keyframe_cur = std::make_shared<KeyFrame>(cur_frame_);
  
  // Insert the two keyframes to map
  //map_.AddKeyFrame(keyframe_ini);
  map_->AddKeyFrame(keyframe_ini);
  map_->AddKeyFrame(keyframe_cur);
  
  // Create landmarks and asscioate to keyframes
  // Triangulate keypoints to get landmarks
  cv::Mat world_coordinates;
  cv::triangulatePoints(K*T_cw_prev.rowRange(0,3), K*T_cw_cur.rowRange(0,3), src_points, dst_points, world_coordinates);
  // Convert OpenCV type CV_32F to CV_64F
  world_coordinates.convertTo(world_coordinates, CV_64F);
  for (size_t i = 0; i < src_points.size(); ++i ) {
    // TODO:: check landmark valid
    Eigen::Vector4d eigen_world_coor;
    cv::Mat x = world_coordinates.col((int)i);
    x /= x.at<double>(3,0);

    // Create landmark
    std::shared_ptr<lslam::Landmark> landmark = std::make_shared<lslam::Landmark>();
    landmark->set_pt_world(x.rowRange(0,3));
    std::cout<<"Landmark "<<i<<" \n"<<landmark->pt_world()<<std::endl;
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
    map_->AddLandmarkPoint(landmark);
    
    // Associate landmark to frame
    last_frame_->set_landmark( good_matches[i].queryIdx, landmark);
    cur_frame_->set_landmark(good_matches[i].trainIdx, landmark);
  }

  return true;
}

void Frontend::DataAssociation() {
  // Track
  bool tracked = false;
  tracked = TrackToLastFrame();
  CHECK(tracked) << "Track failed";
}

// TODO: method as a new class
bool Frontend::TrackToLastFrame() {
  // No last frame exists
  if (!last_frame_) return false;
  // We do not have a predicted velocity
  if (!(last_frame_->T_cl().data)) return false;

  // Initialize current frame pose using predicted velocity
  // We assume camera as a const velocity model
  cur_frame_->SetPose(last_frame_->T_cl()*last_frame_->T_cw()); 
  // Perform active ORB searching and matching, we get 3d-2d matches
  // TUNE: search range
  int th = 15; 
  int n_matches = guided_matcher_.ProjectionGuided3D2DMatcher(cur_frame_, last_frame_, th, true);
  // TUNE: 20?
  if (n_matches < 20) return false;
  std::cout<<"Before optimization: \n"<<cur_frame_->T_cw()<<std::endl;
  ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);
  std::cout<<"After optimization: \n"<<cur_frame_->T_cw()<<std::endl;
  
}

void Frontend::Process(std::shared_ptr<Frame> cur_frame) {

  cur_frame_ = cur_frame;
  
  // Extract ORB features
  cur_frame_->PreProcess(orb_extractor_, camera_model_);

  if (state_ == FrontEndState::kInitialized) {
    // State: initialized  
    // We track the incoming frames
    DataAssociation();

  } else if (state_ == FrontEndState::kNotInitialized) {
    // State: not initialized
    // We try to initialize the camera pose  
    bool init_success = DataAssociationBootstrap();
    if (init_success) {
      state_ = FrontEndState::kInitialized;
      LOG(INFO) << "Initialized!";
    }
  } else if (state_ == FrontEndState::kNotReady) {
    // Steate: not ready
    // System just start, frames received is less than 2
    // For monocular case, we need two frames for initialization
    if (last_frame_)
      state_ = FrontEndState::kNotInitialized;
  }

  last_frame_ = cur_frame_;
}

void Frontend::init(const ParametersReader& params) {
  // Initialize camera model
  // TODO: Check pinholecamera_params != null
  // Smart pointer initialize
  camera_model_ = std::make_shared<PinholeCamera>(params.pinholecamera_params());

  // Initialize ORB extractor
  // TODO: using params to initialize
  int features = 1000;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  orb_extractor_ = std::make_shared<ORB_SLAM2::ORBextractor>(features, scale, level, ini, min);
  
  // Initialize guided matcher
  guided_matcher_.set_camera_model(camera_model_);
  guided_matcher_.set_orb_extractor(orb_extractor_);
}

void Frontend::set_map(std::shared_ptr<Map> map) {
  map_ = map;
}

void Frontend::set_camera_model(std::shared_ptr<PinholeCamera> camera_model) {
  camera_model_ = camera_model;
}



} // namespace lslam
