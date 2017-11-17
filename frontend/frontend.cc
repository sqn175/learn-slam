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
#include <mutex>

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
  , state_(FrontEndState::kNotInitialized)
  , max_keyframe_interval_(30)
  , min_keyframe_interval_(0){
                    
}

void Frontend::Process(cv::Mat image, double timestamp) {
  
  auto start = std::chrono::high_resolution_clock::now();
  cur_frame_ = std::make_shared<Frame>(image, timestamp, orb_extractor_, camera_model_, orb_voc_);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Frame preprocess: " << elapsed.count() << " ms" << std::endl;

  image_ = image.clone();
  cv::cvtColor(image_, image_, CV_GRAY2BGR);
  
    // Get Map Mutex -> Map cannot be changed
  std::unique_lock<std::mutex> lock(map_->mMutexMapUpdate);

  if (state_ == FrontEndState::kTracking) {
    bool tracked = DataAssociation(); 
    if (!tracked) {
      state_ = FrontEndState::kLost;
    } 
  }
  else if (state_ == FrontEndState::kInitialized) {
    // State: initialized  
    state_ = Frontend::kTracking;
    bool tracked = DataAssociation();
    if (!tracked) {
      state_ = FrontEndState::kLost;
    }
  } else if (state_ == FrontEndState::kNotInitialized) {
    // State: not initialized
    // We try to initialize the camera pose  
    bool init_success = DataAssociationBootstrap();
    if (init_success) {
      state_ = FrontEndState::kInitialized;
      LOG(INFO) << "Initialized!";
    }
  }

  // Update tracking variables
  last_frame_ = cur_frame_;
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
  std::vector<cv::DMatch> matches = guided_matcher_->Guided2D2DMatcher(cur_frame_, 100, 50, 0.9, true);
  
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

  std::cout << "New Map created with " << matches.size() << " points" <<std::endl;
  LOG(INFO) << "New Map created with " << matches.size() << " points";
  // Create Map
  // - Wrap keyframes and insert in the map
  init_keyframe_ = std::make_shared<KeyFrame>(*init_frame_, map_);
  cur_keyframe_ = std::make_shared<KeyFrame>(*cur_frame_, map_);
  // init_keyframe->ComputeBoW();
  // cur_keyframe->ComputeBoW();
  map_->AddKeyFrame(init_keyframe_);
  map_->AddKeyFrame(cur_keyframe_);

  // - Wrap mappoints and insert in the map
  for (auto& match : matches) {
    cv::Mat point_3d(points_3d[match.queryIdx]);
    // Wrap CV_32F to CV_64F
    point_3d.convertTo(point_3d, CV_64F);
    auto mappoint = std::make_shared<MapPoint>(point_3d, cur_keyframe_, map_);

    // Associate mappoint to frame
    init_frame_->set_mappoint(match.queryIdx, mappoint);
    cur_frame_->set_mappoint(match.trainIdx, mappoint);

    init_frame_->set_outlier(match.queryIdx, false);
    cur_frame_->set_outlier(match.trainIdx, false);

    // Associate mappoint to keyframe
    init_keyframe_->set_mappoint(match.queryIdx, mappoint);
    cur_keyframe_->set_mappoint(match.trainIdx, mappoint);

    init_keyframe_->set_outlier(match.queryIdx, false);
    cur_keyframe_->set_outlier(match.trainIdx, false);

    // Associate keyframe to mappoint
    mappoint->AddObservation(init_keyframe_, match.queryIdx);
    mappoint->AddObservation(cur_keyframe_, match.trainIdx);

    map_->AddMapPoint(mappoint);
  }

  // Associate keyframe to keyframe
  init_keyframe_->SetConnectedKeyFrames();
  cur_keyframe_->SetConnectedKeyFrames();

  // Bundle Adjustment
  ORB_SLAM2::Optimizer::GlobalBundleAdjustment(map_, 20);
  // Set median depth to 1, and scale
  double median_depth = init_keyframe_->SceneDepth(2);
  CHECK(median_depth >= 0) << "Wrong initialization.";
  // TODO: check there are 100 more mappoints in map, otherwise reset

  // Scale initial pose
  cv::Mat T_cw_scaled = cur_keyframe_->T_cw();
  T_cw_scaled.col(3).rowRange(0,3) = T_cw_scaled.col(3).rowRange(0,3) / median_depth;
  cur_keyframe_->SetPose(T_cw_scaled);

  cur_frame_->SetPose(T_cw_scaled);

  // Scale initial mappoints
  std::vector<std::shared_ptr<MapPoint>> mappoints = init_keyframe_->mappoints();
  for (auto& mp:mappoints) {
    if (mp) {
      mp->set_pt_world(mp->pt_world() / median_depth);
      mp->SetDescriptors();
      mp->SetNormalAndDepth();
    }
  }

  // Set frontend last_keyframe
  reference_keyframe_ = cur_keyframe_;
  last_frame_id_as_kf_ = cur_frame_->id();
  return true;

}

bool Frontend::DataAssociation() {
  // Update last_frame corresponding mappoints.
  // We may replaced some mappoints in mapper thread.
  auto mps = last_frame_->mappoints();
  for (size_t i = 0; i < mps.size(); ++i) {
    auto mp = mps[i];
    if (mp && mp->replaced_mp()) {
      last_frame_->set_mappoint(i,mp->replaced_mp());
    }
  }

  // Track
  bool tracked = TrackToLastFrame();
  if (!tracked) {
    tracked = TrackToLastKeyFrame();
  }
  if (!tracked)
    return false;

  tracked = TrackToLocalMap();

  if (tracked) {
    T_curl_ = cur_frame_->T_cw() * last_frame_->T_wc();
    T_lr_ = cur_frame_->T_cw() * reference_keyframe_->T_wc();
  } else {
    T_curl_ = cv::Mat();
    T_lr_ = cv::Mat();
  }
  
  CHECK(tracked) << "Track failed";
  return tracked;
}

// TODO: method as a new class
bool Frontend::TrackToLastFrame() {
  // We do not have a predicted velocity
  if (!T_curl_.data) 
    return false;

  // Reference keyframe may have optimized its pose,
  // so we need to update pose of last frame.
  if (!T_lr_.data) 
    return false;
  last_frame_->SetPose(T_lr_*reference_keyframe_->T_cw());
  // Now we set an rough initial pose of current frame according to constant velocity model
  cur_frame_->SetPose(T_curl_*last_frame_->T_cw());

  // TUNE:
  int th = 15;
  auto matches = guided_matcher_->ProjectionGuided3D2DMatcher(last_frame_,
                                                              cur_frame_, 
                                                              th, GuidedMatcher::kRadiusFromOctave,
                                                              false, 100, true);

  // TUNE:
  if (matches.size() < 20) {
    // Match with a wider window
    matches = guided_matcher_->ProjectionGuided3D2DMatcher(last_frame_,
                                                          cur_frame_, 
                                                          2*th, GuidedMatcher::kRadiusFromOctave,
                                                          false, 100, true);
  }

  // Few matches, track failed.
  if (matches.size() < 20) {
    return false;
  }

  // Assign 3d mappoints of last to cur_frame_ according to matches
  for(auto& match : matches) {
    cur_frame_->set_mappoint(match.trainIdx, last_frame_->mappoint(match.queryIdx));
  }

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

bool Frontend::TrackToLastKeyFrame() {
  auto matches = guided_matcher_->DbowGuided2D2DMatcher(reference_keyframe_, cur_frame_, 50, true, 0.7);
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
  cur_frame_->SetConnectedKeyFrames();
  // TODO: consider whether we should store the connected mappoints in the Frame class
  // We do not search the mappoints already associated to cur_frame keypoints 
  // in the previous TrackToLastFrame or TrackToLastKeyFrame functions
  auto connected_mappoints = cur_frame_->connected_mappoints();
  auto mappoints = std::vector<std::shared_ptr<MapPoint>>(connected_mappoints.begin(), connected_mappoints.end());

  // Step2. 3d2d match
  std:;vector<size_t> projectable_indices;
  auto matches = guided_matcher_->ProjectionGuided3D2DMatcher(mappoints, cur_frame_, 1, GuidedMatcher::kRadiusFromViewCosine,
                                                             false, 100, 0.8, projectable_indices);
  // Update mappoint statics
  auto mps = cur_frame_->mappoints();
  for (size_t i = 0; i < mps.size(); ++i) {
    auto mp = mps[i];
    if (mp) {
      if (!mp->is_bad())
        mp->IncreaseCntProjected();
      else {
        cur_frame_->EraseMapPoint(i);
      }
    }
  }
  for (auto& idx : projectable_indices) {
    mappoints[idx]->IncreaseCntProjected();
  }

  // Assign matched 3d points to cur_frame
  for (auto& match : matches) {
    // Skip the already associated mappoint
    if (cur_frame_->mappoint(match.trainIdx))
      continue;
    cur_frame_->set_mappoint(match.trainIdx, mappoints[match.queryIdx]);
  }
  
  // Optimize Pose
  ORB_SLAM2::Optimizer::PoseOptimization(cur_frame_);
  
  n_match_to_localmap_ = 0;
  for (size_t i = 0; i < cur_frame_->mappoints().size(); ++i) {
    if (cur_frame_->mappoint(i)) {
      if (cur_frame_->outlier(i)) {
        // We do nothing? Do we need to kick out the outlier mappoint?
      } else {
        cur_frame_->mappoint(i)->IncreaseCntTracked();
        n_match_to_localmap_++;
      }
    }
  }

  // TUNE: 30
  bool success = n_match_to_localmap_ >= 30;
  return success;

}

bool Frontend::FrameIsKeyFrame() {
  // TUNE:
  int th_obs = 3;
  if (map_->SizeOfKeyframes() <= 2) 
    th_obs = 2;

  int n_match_reference_kf = 0;
  auto mps = reference_keyframe_->mappoints();
  for (auto& mp : mps) {
    if (mp && !mp->is_bad() && mp->SizeOfObs() >= th_obs)
      ++n_match_reference_kf;
  }

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1_a = cur_frame_->id() >= last_frame_id_as_kf_ + max_keyframe_interval_;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1_b = cur_frame_->id() >= last_frame_id_as_kf_ + min_keyframe_interval_;
  //Condition 1c: tracking is weak
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  // TUNE: 0.9
  const bool c2 = n_match_to_localmap_ < n_match_reference_kf * 0.9;
  return c2; 
}

void Frontend::CreateKeyFrame() {
  cur_keyframe_ = std::make_shared<KeyFrame>(*cur_frame_, map_);
  reference_keyframe_ = cur_keyframe_;
  last_frame_id_as_kf_ = cur_frame_->id();
  T_lr_ = cv::Mat::eye(4,4,CV_64F);
}

void Frontend::set_map(std::shared_ptr<Map> map) {
  map_ = map;
}

void Frontend::set_camera_model(std::shared_ptr<PinholeCamera> camera_model) {
  camera_model_ = camera_model;
}

} // namespace lslam
