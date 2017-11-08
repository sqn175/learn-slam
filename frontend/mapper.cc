/*
 * @Author: Shi Qin 
 * @Date: 2017-10-25 09:58:15 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 20:35:09
 */

#include "mapper.h"
#include <iostream>
#include <glog/logging.h>
#include <set>

#include "keyframe.h"
#include "mappoint.h"
#include "map.h"
#include "cv.h"
#include "guided_matcher.h"

#include "Optimizer.h"
#include "../3rdparty/ORB_SLAM2_modified/ORBextractor.h"

namespace lslam {

Mapper::Mapper(std::shared_ptr<Map> map,
                std::shared_ptr<GuidedMatcher> guided_matcher,
                std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor) 
  : map_(map)
  , guided_matcher_(guided_matcher) {

  scale_levels_ = orb_extractor->GetLevels();
  scale_factor_ = orb_extractor->GetScaleFactor();
  log_scale_factor_ = std::log(scale_factor_);
  scale_factors_ = orb_extractor->GetScaleFactors();
  inv_scale_factors_ = orb_extractor->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor->GetInverseScaleSigmaSquares();
  max_level_ = orb_extractor->GetLevels();  
}

void Mapper::Process(std::shared_ptr<KeyFrame> keyframe) {
  cur_keyframe_ = keyframe;
  InsertKeyFrame(keyframe);
  CullMapPoints(keyframe);
  TriangulateNewMapPoints(keyframe);
  if (map_->SizeOfKeyframes() <= 2) {
    return;
  }
  FuseAndAssociateMapPoints(keyframe);
  // ORB_SLAM may set mbAbortBA as true to stop local BA
  // TODO: consider if we need this mechanism
  bool mbAbortBA = false;
  ORB_SLAM2::Optimizer::LocalBundleAdjustment(keyframe, map_);

  CullKeyFrames(keyframe);
}

void Mapper::InsertKeyFrame(std::shared_ptr<KeyFrame> keyframe){
  // Compute Bow
  keyframe->ComputeBoW();

  // Associate the keyframe to mappoints by adding observation
  auto mappoints = keyframe->mappoints();
  for (size_t i = 0; i < mappoints.size(); ++i) {
    std::shared_ptr<MapPoint> mp = mappoints[i];
    if (mp && !mp->is_bad()) {
      if (!mp->IsObservedByKeyFrame(keyframe)) {
        mp->AddObservation(keyframe, i);
        mp->SetNormalAndDepth();
        mp->SetDescriptors();
      }
    }
  }

  // Associate the keyframe to already existing keyframes in the map
  keyframe->ConnectToMap();

  // Insert the keyframe into the map
  map_->AddKeyFrame(keyframe);
}

void Mapper::CullMapPoints(std::shared_ptr<KeyFrame> keyframe) {
  auto it = candidate_mappoints_.begin();
  // TUNE:
  const int obs_th = 2;
  while (it != candidate_mappoints_.end()) {
    auto mp = *it;
    if (mp->is_bad()) {
      // A mappoint is set to be bad in function SetBadFlag() and Replace()
      // Replace() is called in FuseAndAssociateMapPoints()
      // SetBadFlag() is called when there is too few observations after erasing some observations in optimization process
      it = candidate_mappoints_.erase(it);
    } else if (mp->TrackedRatio() < 0.25) {
      mp->SetBadFlag();
      map_->EraseMapPoint(mp);
      it = candidate_mappoints_.erase(it);
    } else if ( ((int)keyframe->id() - (int)mp->created_by_keyframe_id())>= obs_th && mp->SizeOfObs() <= obs_th ) {
      mp->SetBadFlag();
      map_->EraseMapPoint(mp);
      it = candidate_mappoints_.erase(it);
    } else if ( ((int)keyframe->id() - (int)mp->created_by_keyframe_id()) >= obs_th + 1 ) {
      it = candidate_mappoints_.erase(it);
    } else {
      ++it;
    }
  }
}

void Mapper::TriangulateNewMapPoints(std::shared_ptr<KeyFrame> keyframe) {
  // TUNE: 20
  auto neighbors_of_kf = keyframe->GetConnectedKeyFrames(20);
  // Keyframe Rotation matrix inverse
  cv::Mat R_wc_kf = keyframe->R_cw().t();
  cv::Mat T_cw_kf = keyframe->T_cw();
  cv::Mat o_w_kf = keyframe->o_w();
  // TUNE:
  const double ratio_factor = 1.5 * scale_factor_;
  // Match keyframe to its every neighbor with epipolar restriction
  // and triangulate new mappoints according to searched matches
  for (auto& neighbor : neighbors_of_kf) {
    cv::Mat R_wc_nei = neighbor->R_cw().t();
    cv::Mat T_cw_nei = neighbor->T_cw();
    cv::Mat o_w_nei = neighbor->o_w();
    // Check first that the baseline is not too short
    double baseline = cv::norm(neighbor->o_w() - o_w_kf);
    double scene_depth_neighbor = neighbor->SceneDepth(2);
    double ratio = baseline / scene_depth_neighbor;
    // TUNE: 0.01
    if (ratio < 0.01)
      continue;

    cv::Mat F = RecoverFundamental(keyframe, neighbor);

    // Search match
    auto matches = guided_matcher_->DbowAndEpipolarGuided2D2DMatcher(keyframe, neighbor, 50, false, false);
    
    // Triangulate each match
    // TODO: can I use cv::TriangulatePoints() first, then check parallax?
    // 1 ->ewhbor
    cv::Mat p3d(4,1,CV_64F);
    // Preallocate SVD matrices on stack
    cv::Mat a(4,4,CV_64F);
    cv::Mat w(4,1,CV_64F);
    cv::Mat u(4,4,CV_64F);
    cv::Mat v(4,4,CV_64F);
    for (auto& match : matches) {
      // Check parallax between rays
      const cv::KeyPoint& kp_kf = keyframe->undistorted_kp(match.queryIdx);
      const cv::KeyPoint& kp_nei = neighbor->undistorted_kp(match.trainIdx);

      cv::Mat x_c_kf = keyframe->camera_model()->UnProject(kp_kf);
      cv::Mat x_c_nei = neighbor->camera_model()->UnProject(kp_nei);

      cv::Mat ray_kf = R_wc_kf * x_c_kf;
      cv::Mat ray_nei = R_wc_nei * x_c_nei;

      const double parallax_ray_cosine = ray_kf.dot(ray_nei) / (cv::norm(ray_kf) * cv::norm(ray_nei));

      // TUNE:
      if (parallax_ray_cosine > 0 && parallax_ray_cosine < 0.9998) {
        //Compute the position of a 3D point seen from two viewpoints. Linear Method.
        a.row(0) = x_c_kf.at<double>(0)*T_cw_kf.row(2) - T_cw_kf.row(0);
        a.row(1) = x_c_kf.at<double>(1)*T_cw_kf.row(2) - T_cw_kf.row(1);
        a.row(2) = x_c_nei.at<double>(0)*T_cw_nei.row(2) - T_cw_nei.row(0);
        a.row(3) = x_c_nei.at<double>(1)*T_cw_nei.row(2) - T_cw_nei.row(1);

        cv::SVD::compute(a,w,u,v,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        p3d = v.row(3).t();

        CHECK(p3d.at<double>(3)!=0);

        // Euclidean coordinates
        p3d = p3d.rowRange(0,3) / p3d.at<double>(3);
      } else {
        continue;
      }

      // Check positive depth
      double depth_kf = keyframe->Depth(p3d);
      if (depth_kf <= 0) 
        continue;

      double depth_nei = neighbor->Depth(p3d);
      if (depth_nei <= 0)
        continue;

      // Check reprojection error
      cv::Mat uv = keyframe->KRtProject(p3d);
      double d_u = uv.at<double>(0) - kp_kf.pt.x;
      double d_v = uv.at<double>(1) - kp_kf.pt.y;
      // TUNE:
      double error_th = 5.991 * level_sigma2_[kp_kf.octave];
      if ((d_u*d_u + d_v*d_v) > error_th)
        continue;

      uv = neighbor->KRtProject(p3d);
      d_u = uv.at<double>(0) - kp_nei.pt.x;
      d_v = uv.at<double>(1) - kp_nei.pt.y;
      error_th = 5.991 * level_sigma2_[kp_nei.octave];
      if ((d_u*d_u + d_v*d_v) > error_th)
        continue;

      // Check scale consistency
      double dist_kf = cv::norm(p3d - o_w_kf);
      double dist_nei = cv::norm(p3d - o_w_nei);

      if (dist_kf == 0 || dist_nei ==0)
        continue;

      const double ratio_dist = dist_nei / dist_kf;
      const double ratio_octave = scale_factors_[kp_kf.octave] / scale_factors_[kp_nei.octave];
      // TODO: why?
      if (ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave*ratio_factor)
        continue;

      // This triangulated 3d point is valid, we create a new mappoint
      auto mappoint = std::make_shared<MapPoint>(p3d, keyframe);

      mappoint->AddObservation(keyframe, match.queryIdx);
      mappoint->AddObservation(neighbor, match.trainIdx);

      keyframe->set_mappoint(match.queryIdx, mappoint);
      neighbor->set_mappoint(match.trainIdx, mappoint);

      mappoint->SetDescriptors();
      mappoint->SetNormalAndDepth();

      map_->AddMapPoint(mappoint);
      candidate_mappoints_.push_back(mappoint);

      // we will finally optimize these new mappoints
    }
    
  }
}

void Mapper::FuseAndAssociateMapPoints(std::shared_ptr<KeyFrame> keyframe) {
  // Retrive neighbor keyframes
  // TUNE: 20
  std::set<std::shared_ptr<KeyFrame>> extended_connected_keyframes;
  auto neighbors_of_kf = keyframe->GetConnectedKeyFrames(20);
  for (auto& nei : neighbors_of_kf) {
    if (nei->is_bad())
      continue;

    extended_connected_keyframes.insert(nei);

    // TUNE: 5
    auto neighbors_of_nei = nei->GetConnectedKeyFrames(5);
    for (auto& nei_of_nei : neighbors_of_nei) {
      if (nei_of_nei->is_bad() || nei_of_nei->id() == keyframe->id())
        continue;

      extended_connected_keyframes.insert(nei_of_nei);
    }
  }

  // Search matches by projecting keyframe associated mappoints to neighbor keyframes 
  for (auto& nei : extended_connected_keyframes) {

    auto matches = guided_matcher_->ProjectionGuided3D2DMatcher(keyframe->mappoints(),
                                                                nei, 3.0, GuidedMatcher::kRadiusFromOctave,
                                                                true, 50, 0);

    for (auto& match : matches) {
      auto mp_kf = keyframe->mappoint(match.queryIdx);
      if (!mp_kf) // mp_kf may be erased in previous loop 
        continue;
      auto mp_nei = nei->mappoint(match.trainIdx);
      if (mp_nei) { // Replace, TODO: why replace?
        if (mp_nei->is_bad())
          continue;
        if (mp_nei->SizeOfObs() > mp_kf->SizeOfObs()) {
          if (mp_kf->ReplaceWith(mp_nei))
            map_->EraseMapPoint(mp_kf);
        } else {
          if (mp_nei->ReplaceWith(mp_kf))
            map_->EraseMapPoint(mp_nei);
        }
      } else { // Associate a new mappoint
        mp_kf->AddObservation(nei, match.trainIdx);
        nei->set_mappoint(match.trainIdx, mp_kf);
      }
    }
  }

  // Search matches by projection all neighbor keyframe associated mappoints to current keyframe
  // kind of like the process of TrackLocalMap()
  std::set<std::shared_ptr<MapPoint>> connected_mappoints;
  for (auto& nei : extended_connected_keyframes) {
    auto mps_of_nei = nei->mappoints();
    for(auto& mp : mps_of_nei) {
      if (!mp || mp->is_bad()) 
        continue;

      connected_mappoints.insert(mp);
     }
  }
  std::vector<std::shared_ptr<MapPoint>> mps(connected_mappoints.begin(), connected_mappoints.end());
  auto matches = guided_matcher_->ProjectionGuided3D2DMatcher(mps, keyframe, 
                                                              3.0, GuidedMatcher::kRadiusFromOctave,
                                                              true, 50, false);

  
  for (auto& match : matches) {
    auto mp_kf = keyframe->mappoint(match.trainIdx);
    auto mp_nei = mps[match.queryIdx];
    if (mp_kf) { // Replace, TODO: why replace?
      if (mp_kf->is_bad())
        continue;
      if (mp_kf->SizeOfObs() > mp_nei->SizeOfObs()) {
        if (mp_nei->ReplaceWith(mp_kf))
          map_->EraseMapPoint(mp_nei);
      } else {
        if (mp_kf->ReplaceWith(mp_nei))
          map_->EraseMapPoint(mp_kf);
      }
    } else { // Associate a new mappoint
      mp_nei->AddObservation(keyframe, match.trainIdx);
      keyframe->set_mappoint(match.trainIdx, mp_nei);
    }
  }

  // Update current keyframe associated mappoints
  auto mps_kf = keyframe->mappoints();
  for (auto& mp : mps_kf) {
    if (mp && !mp->is_bad()) {
      mp->SetDescriptors();
      mp->SetNormalAndDepth();
    }
  }

  // Update connections in covisibility graph
  keyframe->ConnectToMap();

}

void Mapper::CullKeyFrames(std::shared_ptr<KeyFrame> keyframe) {
  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
  // in at least other 3 keyframes (in the same or finer scale)
  // We only consider close stereo points
  // TUNE:
  const int obs_th = 3;
  auto local_keyframes = keyframe->GetConnectedKeyFrames();
  for (auto& lkf : local_keyframes) {
    if (lkf->id() == 0)
      continue;
    const auto mappoints = lkf->mappoints();
    int n_mps = 0;
    int n_redundant = 0;
    for (size_t i = 0; i < mappoints.size(); ++i) {
      auto& mp = mappoints[i];
      if (mp && !mp->is_bad()) {
        ++n_mps;
        if (mp->SizeOfObs() > obs_th) {
          const int& octave = lkf->undistorted_kp(i).octave;
          auto obs = mp->observations();
          int n_obs = 0;

          for (auto& ob : obs) {
            auto ob_kf = ob.first;
            if (ob_kf == lkf)
              continue;
            const int& octave_ob = ob_kf->undistorted_kp(ob.second).octave;
            if (octave_ob <= octave +1) {
              n_obs++;
              if (n_obs >= obs_th)
                break;
            }
          }
          if (n_obs >= obs_th) {
            ++n_redundant;
          }
        }
      }
    }

    if (n_redundant > 0.9*n_mps) {
      lkf->SetBadFlag();
      map_->EraseKeyFrame(lkf);
    }

  }
}

} // namespace lslam
