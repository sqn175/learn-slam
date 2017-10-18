/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>


#include "frontend.h"

#include "opencv2/core/eigen.hpp"

#include "helper.h"

#include "frame.h"
#include "keyframe.h"
#include "map.h"
#include "mappoint.h"

namespace lslam {

Frontend::Frontend() : state_(FrontEndState::kNotInitialized) {
}
/*
bool Frontend::DataAssociationBootstrap() {
  
  // Find correspondences
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> matches;

  auto start = std::chrono::high_resolution_clock::now();
  // KNN matcher, k = 2, find the best match and the second best match
  matcher.knnMatch(last_frame_->descriptors(), cur_frame_->descriptors(), matches, 2);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Brute-force match: " << elapsed.count() << " ms" << std::endl;
  
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
  cv::Mat T_cw_scaled = cv::Mat::eye(4,4,CV_64F);
  R.copyTo(T_cw_scaled.rowRange(0,3).colRange(0,3));
  t.copyTo(T_cw_scaled.rowRange(0,3).col(3));
  cur_frame_->SetPose(T_cw_scaled);
  cur_frame_->set_T_cl(T_cw_scaled*last_frame_->T_wc());

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
  cv::triangulatePoints(K*T_cw_prev.rowRange(0,3), K*T_cw_scaled.rowRange(0,3), src_points, dst_points, world_coordinates);
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
*/

bool Frontend::DataAssociationBootstrap() {
  // Step1. Set a initial reference frame
  if (!init_frame_) {
    // TUNE: 100
    if (cur_frame_->keypoints().size() >= 100) {
      // Assign initial reference frame
      init_frame_ = cur_frame_;
      // Setup the initial frame of 2d2d matcher
      guided_matcher_.SetupGuided2D2DMatcher(init_frame_);
    } 
    return false;
  }
  
  // Now we have a initial reference frame
  // We ensure the incoming frame have enough keypoints
  // TUNE: 100
  if (cur_frame_->keypoints().size() < 100) {
    // delete initial reference frame
    init_frame_.reset();
    return false;
  }

  // Step2. Find correspondences
  std::vector<cv::DMatch> matches;

  int n_match = guided_matcher_.Guided2D2DMatcher(cur_frame_, matches, 100, 50, true, 0.9, true);

  // Check if we have enough correspondences
  // TUNE: 100
  if (n_match < 100) {
    init_frame_.reset();
    return false;
  }
  
  // Step3. Recover pose from correspondences
  cv::Mat R_cw; // CV_32F current camera rotation
  cv::Mat t_cw;
  std::vector<cv::Point3f> points_3d; // Triangulated points
  std::vector<bool> is_triangulated;  // Triangulated Correspondences
  bool initialized = ComputeEgomotion(init_frame_, cur_frame_, matches, R_cw, t_cw, points_3d, is_triangulated);
  if (!initialized) 
    return false;

  // Step4. Initialize map from recovered pose and triangulated points
  // Update matches according to triangulation results
  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i].trainIdx >= 0 && !is_triangulated[i]) {
      matches[i].trainIdx = -1;
      n_match--;
    }
  }

  // Set frame pose
  // Wrap R_cw and t_cw to CV_64
  R_cw.convertTo(R_cw, CV_64F);
  t_cw.convertTo(t_cw, CV_64F);
  init_frame_->SetPose(cv::Mat::eye(4,4,CV_64F));
  cv::Mat T_cw = cv::Mat::eye(4,4,CV_64F);
  R_cw.copyTo(T_cw.rowRange(0,3).colRange(0,3));
  t_cw.copyTo(T_cw.rowRange(0,3).col(3));
  cur_frame_->SetPose(T_cw);

  // Create Map
  // - Wrap keyframes and insert in the map
  auto init_keyframe = std::make_shared<KeyFrame>(*init_frame_);
  auto cur_keyframe = std::make_shared<KeyFrame>(*cur_frame_);
  map_->AddKeyFrame(init_keyframe);
  map_->AddKeyFrame(cur_keyframe);

  // - Wrap mappoints and insert in the map
  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i].trainIdx < 0)
      continue;
    
    cv::Mat point_3d(points_3d[i]);
    // Wrap CV_32F to CV_64F
    point_3d.convertTo(point_3d, CV_64F);
    auto mappoint = std::make_shared<MapPoint>(point_3d);
    // Insert in the map
    map_->AddMapPoint(mappoint);

    // Associate mappoint to frame
    init_frame_->set_mappoint(matches[i].queryIdx, mappoint);
    cur_frame_->set_mappoint(matches[i].trainIdx, mappoint);

    init_frame_->set_outlier(matches[i].queryIdx, false);
    cur_frame_->set_outlier(matches[i].trainIdx, false);

    // Associate mappoint to keyframe
    init_keyframe->set_mappoint(matches[i].queryIdx, mappoint);
    cur_keyframe->set_mappoint(matches[i].trainIdx, mappoint);

    init_keyframe->set_outlier(matches[i].queryIdx, false);
    cur_keyframe->set_outlier(matches[i].trainIdx, false);

    // Associate keyframe to mappoint
    mappoint->AddObservation(init_keyframe, matches[i].queryIdx);
    mappoint->AddObservation(cur_keyframe, matches[i].trainIdx);

    mappoint->ComputeDistinctiveDescriptors();
  }

  // Associate keyframe to keyframe
  init_keyframe->UpdateConnections();
  cur_keyframe->UpdateConnections();

  LOG(INFO) << "New Map created with " << map_->SizeOfMappoints() << " points";

  // Bundle Adjustment
  ORB_SLAM2::Optimizer::GlobalBundleAdjustemnt(map_, 20);

  // Set median depth to 1, and scale
  float median_depth = init_keyframe->SceneDepth(2);
  CHECK(median_depth >= 0) << "Wrong initialization.";

  // Scale initial pose
  cv::Mat T_cw_scaled = cur_keyframe->T_cw();
  T_cw_scaled.col(3).rowRange(0,3) = T_cw_scaled.col(3).rowRange(0,3) / median_depth;
  cur_keyframe->SetPose(T_cw_scaled);

  cur_frame_->SetPose(T_cw_scaled);

  // Scale initial mappoints
  std::vector<std::shared_ptr<MapPoint>> mappoints = init_keyframe->mappoints();
  for (auto& mp:mappoints) {
    if (mp) {
      mp->set_pt_world(mp->pt_world() / median_depth);
    }
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
  // No last frame
  if (!last_frame_) return false;
  // We do not have a predicted velocity
  if (!(last_frame_->T_cl().data)) return false;

  // Initialize current frame pose using predicted velocity
  // We assume camera as a const velocity model
  cur_frame_->SetPose(last_frame_->T_cl()*last_frame_->T_cw()); 
  // Perform active ORB searching and matching, we get 3d-2d matches
  // TUNE: search range
  int th = 15; 
  auto start = std::chrono::high_resolution_clock::now();
  int n_matches = guided_matcher_.ProjectionGuided3D2DMatcher(cur_frame_, last_frame_, th, true);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Guided match: " << elapsed.count() << " ms" << std::endl;
  // TUNE: 20?
  if (n_matches < 20) return false;

  start = std::chrono::high_resolution_clock::now();

  std::cout<<"Before optimization: \n"<<cur_frame_->T_cw()<<std::endl;
  ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);
  std::cout<<"After optimization: \n"<<cur_frame_->T_cw()<<std::endl;

  end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Motion-only optimization: " << elapsed.count() << " ms" << std::endl;
  
  return true;
}

void Frontend::Process(std::shared_ptr<Frame> cur_frame) {

  cur_frame_ = cur_frame;
  
  auto start = std::chrono::high_resolution_clock::now();
  // Extract ORB features
  cur_frame_->PreProcess(orb_extractor_, camera_model_);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Frame preprocess: " << elapsed.count() << " ms" << std::endl;

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
  int features = 2000;
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
