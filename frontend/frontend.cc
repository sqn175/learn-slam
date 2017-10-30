/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <utility>
#include <map>


#include <Optimizer.h>

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

Frontend::Frontend(std::shared_ptr<Map> map,
                   std::shared_ptr<PinholeCamera> camera_model,
                   std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor,
                   std::shared_ptr<ORBVocabulary> orb_voc,
                   std::shared_ptr<GuidedMatcher> guided_matcher) 
  : map_(map)
  , camera_model_(camera_model)
  , orb_extractor_(orb_extractor)
  , orb_voc_(orb_voc)
  , guided_matcher_(guided_matcher)
  , state_(FrontEndState::kNotInitialized) {
                    
}

bool Frontend::DataAssociationBootstrap() {
  // Step1. Set a initial reference frame
  if (!init_frame_) {
    // TUNE: 100
    if (cur_frame_->keypoints().size() >= 100) {
      // Assign initial reference frame
      init_frame_ = cur_frame_;
      // Setup the initial frame of 2d2d matcher
      guided_matcher_->SetupGuided2D2DMatcher(init_frame_);
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
  std::vector<cv::DMatch> matches = guided_matcher_->Guided2D2DMatcher(cur_frame_, 100, 50, true, 0.9, true);
  
  // Draw matches
  for(auto& match : matches) {
    cv::line(image_, init_frame_->keypoint(match.queryIdx).pt, cur_frame_->keypoint(match.trainIdx).pt,
             cv::Scalar(0,255,0));
    cv::circle(image_, cur_frame_->keypoint(match.trainIdx).pt, 2, cv::Scalar(0,255,0));
  }

  // Check if we have enough correspondences
  // TUNE: 100
  if (matches.size()< 100) {
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

  matches.erase(std::remove_if(matches.begin(), matches.end(), 
                                 [&is_triangulated](cv::DMatch& match) -> bool { 
                                  return !is_triangulated[match.queryIdx]; 
                                 }),
                                 matches.end());
 
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
  // init_keyframe->ComputeBoW();
  // cur_keyframe->ComputeBoW();
  map_->AddKeyFrame(init_keyframe);
  map_->AddKeyFrame(cur_keyframe);

  // - Wrap mappoints and insert in the map
  for (auto& match : matches) {
    cv::Mat point_3d(points_3d[match.queryIdx]);
    // Wrap CV_32F to CV_64F
    point_3d.convertTo(point_3d, CV_64F);
    auto mappoint = std::make_shared<MapPoint>(point_3d, cur_keyframe);
    // Insert in the map
    map_->AddMapPoint(mappoint);

    // Associate mappoint to frame
    init_frame_->set_mappoint(match.queryIdx, mappoint);
    cur_frame_->set_mappoint(match.trainIdx, mappoint);

    init_frame_->set_outlier(match.queryIdx, false);
    cur_frame_->set_outlier(match.trainIdx, false);

    // Associate mappoint to keyframe
    init_keyframe->set_mappoint(match.queryIdx, mappoint);
    cur_keyframe->set_mappoint(match.trainIdx, mappoint);

    init_keyframe->set_outlier(match.queryIdx, false);
    cur_keyframe->set_outlier(match.trainIdx, false);

    // Associate keyframe to mappoint
    mappoint->AddObservation(init_keyframe, match.queryIdx);
    mappoint->AddObservation(cur_keyframe, match.trainIdx);

  }

  // Associate keyframe to keyframe
  init_keyframe->ConnectToMap();
  cur_keyframe->ConnectToMap();

  LOG(INFO) << "New Map created with " << map_->SizeOfMappoints() << " points";

  // Bundle Adjustment
  ORB_SLAM2::Optimizer::GlobalBundleAdjustemnt(map_, 20);

  // Set median depth to 1, and scale
  double median_depth = init_keyframe->SceneDepth(2);
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
      mp->ComputeDistinctiveDescriptors();
      mp->UpdateNormalAndDepth();
    }
  }

  // Set frontend last_keyframe
  reference_keyframe_ = cur_keyframe;

  return true;

}

void Frontend::DataAssociation() {
  // Track
  bool tracked = false;
  tracked = TrackToLastKeyFrame();
  if (tracked) {
    TrackToLocalMap();
  }
  CHECK(tracked) << "Track failed";
}

// TODO: method as a new class
bool Frontend::TrackToLastFrame() {
  // // No last frame
  // if (!last_frame_) return false;
  // // We do not have a predicted velocity
  // if (!(last_frame_->T_cl().data)) return false;

  // // Initialize current frame pose using predicted velocity
  // // We assume camera as a const velocity model
  // cur_frame_->SetPose(last_frame_->T_cl()*last_frame_->T_cw()); 
  // // Perform active ORB searching and matching, we get 3d-2d matches
  // // TUNE: search range
  // int th = 15; 
  // auto start = std::chrono::high_resolution_clock::now();
  // std::vector<cv::DMatch> matches = guided_matcher_->ProjectionGuided3D2DMatcher(last_frame_, cur_frame_,th, 100, true, false);
  // auto end = std::chrono::high_resolution_clock::now();
  // std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // std::cout << "Guided match: " << elapsed.count() << " ms" << std::endl;
  // // TUNE: 20?
  // if (matches.size() < 20) return false;

  // start = std::chrono::high_resolution_clock::now();

  // std::cout<<"Before optimization: \n"<<cur_frame_->T_cw()<<std::endl;
  // ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);
  // std::cout<<"After optimization: \n"<<cur_frame_->T_cw()<<std::endl;

  // end = std::chrono::high_resolution_clock::now();
  // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // std::cout << "Motion-only optimization: " << elapsed.count() << " ms" << std::endl;
  
  // return true;
}

bool Frontend::TrackToLastKeyFrame() {
  auto matches = guided_matcher_->DbowGuided2D2DMatcher(reference_keyframe_, cur_frame_, 50, true, true, 0.7);
  // TUNE: 15
  if (matches.size() < 15)
    return false;

  // Assign 3d mappoints of reference_keyframe_ to cur_frame_ according to matches
  for(auto& match : matches) {
    cur_frame_->set_mappoint(match.trainIdx, reference_keyframe_->mappoint(match.queryIdx));
  }
  // Set a rough initial pose
  cur_frame_->SetPose(last_frame_->T_cw());

  // Optimize Pose
  ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);

  int n_match = 0;
  // Kick out the outlier associated mappoint
  for(size_t i = 0; i < cur_frame_->mappoints().size(); ++i) {
    if (cur_frame_->mappoint(i)) {
      if (cur_frame_->outlier(i)) {
        cur_frame_->mappoint(i).reset();
        cur_frame_->set_outlier(i, false);
      } else {
        n_match++;
      }
    }
  }

  // TUNE: 10
  bool success = n_match >= 10;
  return success;
}

bool Frontend::TrackToLocalMap() {
  // Step1. Retrive local map
  //        Local map consists of mappoints and keyframes associated to current frame
  cur_frame_->ConnectToMap();
  // TODO: consider whether we should store the connected mappoints in the Frame class
  auto connected_mappoints = cur_frame_->connected_mappoints();
  auto mappoints = std::vector<std::shared_ptr<MapPoint>>(connected_mappoints.begin(), connected_mappoints.end());

  // Step2. 3d2d match
  auto matches = guided_matcher_->ProjectionGuided3D2DMatcher(mappoints, cur_frame_, 1, 100, true, 0.8);

  // Assign matched 3d points to cur_frame
  for (auto& match : matches) {
    cur_frame_->set_mappoint(match.trainIdx, mappoints[match.queryIdx]);
  }
  
  // Optimize Pose
  ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);
  
  int n_match = 0;
  for (size_t i = 0; i < cur_frame_->mappoints().size(); ++i) {
    if (cur_frame_->mappoint(i)) {
      if (cur_frame_->outlier(i)) {
        // We do nothing? Do we need to kick out the outlier mappoint?
      } else {
        n_match++;
      }
    }
  }

  // TUNE: 30
  bool success = n_match >= 30;
  return success;

}

void Frontend::Process(cv::Mat image, double timestamp) {
  
  auto start = std::chrono::high_resolution_clock::now();
  cur_frame_ = std::make_shared<Frame>(image, timestamp, orb_extractor_, camera_model_, orb_voc_);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Frame preprocess: " << elapsed.count() << " ms" << std::endl;

  image_ = image.clone();
  cv::cvtColor(image_, image_, CV_GRAY2BGR);
  
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


void Frontend::set_map(std::shared_ptr<Map> map) {
  map_ = map;
}

void Frontend::set_camera_model(std::shared_ptr<PinholeCamera> camera_model) {
  camera_model_ = camera_model;
}

void Frontend::PublishVisualization(cv::Mat& im, std::shared_ptr<Frame>& frame){
  im = image_.clone();
  frame = cur_frame_;
}

} // namespace lslam
