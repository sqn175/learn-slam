/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:19:52 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 20:18:11
 */
#include "guided_matcher.h"

#include <algorithm>
#include <iterator>
#include <limits>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include "helper.h"
#include "frame.h"
#include "keyframe.h"
#include "mappoint.h"
#include "cv.h"

namespace lslam
{

const int GuidedMatcher::TH_HIGH = 100;
const int GuidedMatcher::TH_LOW = 50;

GuidedMatcher::GuidedMatcher(std::shared_ptr<PinholeCamera> camera_model,
                             std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor) 
  : camera_model_(camera_model)
  , orb_extractor_(orb_extractor) {

  scale_levels_ = orb_extractor->GetLevels();
  scale_factor_ = orb_extractor->GetScaleFactor();
  log_scale_factor_ = std::log(scale_factor_);
  scale_factors_ = orb_extractor->GetScaleFactors();
  inv_scale_factors_ = orb_extractor->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor->GetInverseScaleSigmaSquares();
  max_level_ = orb_extractor->GetLevels();      
}

std::vector<cv::DMatch> GuidedMatcher::Matcher(const cv::Mat &query_descriptors, const std::vector<size_t> query_indices,
                                               const cv::Mat &train_descriptors, const std::vector<std::vector<size_t>> guided_train_indices,
                                               const float dist_th,
                                               const bool use_ratio_test, const float ratio)
{
  int n = query_descriptors.rows;
  CHECK(n == query_indices.size() && n == guided_train_indices.size());
  CHECK(query_descriptors.cols == train_descriptors.cols);
  // trainIdx equal -1 means this is a invalid matche
  auto matches = std::vector<cv::DMatch>(n, cv::DMatch(-1, -1, std::numeric_limits<float>::max()));
  auto inv_matches = std::vector<cv::DMatch>(train_descriptors.rows, cv::DMatch(-1, -1, std::numeric_limits<float>::max()));

  for (int i = 0; i < n; ++i)
  {

    std::vector<size_t> indices = guided_train_indices[i];

    if (indices.empty())
      continue;

    cv::Mat query_descriptor = query_descriptors.row(i);
    int train_idx = -1;

    // We search in indices
    float min_dist = std::numeric_limits<float>::max();
    float min_dist2 = std::numeric_limits<float>::max();

    for (auto &idx : indices)
    {

      float dist = DescriptorDist(query_descriptor, train_descriptors.row(idx));

      if ( dist <= dist_th && dist < min_dist)
      {
        if (use_ratio_test)
        {
          min_dist2 = min_dist;
        }
        min_dist = dist;
        train_idx = idx;
      }
      else if (use_ratio_test && dist < min_dist2)
      {
        min_dist2 = dist;
      }
    }

    // If the distance is above the dist_th, the match is invalid
    if (min_dist > dist_th)
    {
      matches[i] = cv::DMatch(i, -1, std::numeric_limits<float>::max());
      continue;
    }

    if (use_ratio_test)
    {
      // We apply the ratio test
      if (min_dist >= ratio * min_dist2)
      {
        matches[i] = cv::DMatch(i, -1, std::numeric_limits<float>::max());
        continue;
      }
    }

    // We get a match, it should be one to one
    cv::DMatch inv_match = inv_matches[train_idx];
    if (inv_match.trainIdx >= 0)
    {
      if (inv_match.distance < min_dist)
      {
        matches[i] = cv::DMatch(i, -1, std::numeric_limits<float>::max());
        continue;
      }
    }
    matches[i] = cv::DMatch(i, train_idx, min_dist);
    inv_matches[train_idx] = cv::DMatch(train_idx, i, min_dist);
  }

  std::vector<cv::DMatch> final_matches;
  // Wrap cv::DMatch with query_indices
  for (int i = 0; i < n; ++i)
  {
    auto match = matches[i];
    if (match.trainIdx < 0)
      continue;
    match.queryIdx = query_indices[i];
    final_matches.push_back(match);
  }

  return final_matches;
}

void GuidedMatcher::SetupGuided2D2DMatcher(std::shared_ptr<Frame> query_frame)
{
  // Setup query things
  guided_2d_pts_.clear();
  init_query_descriptors_ = cv::Mat();
  init_query_indices_.clear();

  init_query_frame_ = query_frame;
  const std::vector<cv::KeyPoint> &undistorted_kps = query_frame->undistorted_kps();
  cv::Mat descriptors = query_frame->descriptors();
  for (int i = 0; i < undistorted_kps.size(); ++i)
  {
    // We only search octave 0
    if (undistorted_kps[i].octave != 0)
      continue;

    guided_2d_pts_.push_back(undistorted_kps[i].pt);
    init_query_descriptors_.push_back(descriptors.row(i));
    init_query_indices_.push_back(i);
  }
}

std::vector<cv::DMatch> GuidedMatcher::Guided2D2DMatcher(std::shared_ptr<Frame> train_frame,
                                                         const int radius, const float dist_th,
                                                         const bool use_ratio_test, const float ratio, const bool check_rotation)
{
  // Wrap matcher parameters
  std::vector<std::vector<size_t>> guided_train_indices;
  cv::Mat range = cv::Mat(2, 2, CV_64F);
  for (size_t i = 0; i < guided_2d_pts_.size(); ++i)
  {
    range.at<double>(0, 0) = (double)guided_2d_pts_[i].x - radius;
    range.at<double>(0, 1) = (double)guided_2d_pts_[i].y - radius;
    range.at<double>(1, 0) = (double)guided_2d_pts_[i].x + radius;
    range.at<double>(1, 1) = (double)guided_2d_pts_[i].y + radius;
    std::vector<size_t> indices = train_frame->range_searcher()->PointsInRange(range);
    // Check octave, we erase the keypoint index with octave 0
    indices.erase(std::remove_if(indices.begin(), indices.end(), [&train_frame](size_t& i) -> bool { return train_frame->undistorted_kp(i).octave != 0; }),
                  indices.end());
    guided_train_indices.push_back(indices);
  }

  // We search
  std::vector<cv::DMatch> matches = Matcher(init_query_descriptors_, init_query_indices_,
                                            train_frame->descriptors(), guided_train_indices,
                                            dist_th, use_ratio_test, ratio);

  // Check rotation
  if (check_rotation)
    matches = CheckRotation(init_query_frame_, train_frame, matches);

  // Update guided_2d_pts
  for (auto &match : matches)
  {
    auto idx = lslam::binary_search(init_query_indices_.begin(), init_query_indices_.end(), match.queryIdx);
    int i = std::distance(init_query_indices_.begin(), idx);
    guided_2d_pts_[i] = train_frame->undistorted_kp(match.trainIdx).pt;
  }

  return matches;
}

std::vector<cv::DMatch> GuidedMatcher::ProjectionGuided3D2DMatcher(std::vector<std::shared_ptr<MapPoint>> query_mappoints,
                                                                   std::shared_ptr<Frame> train_frame,
                                                                   const double radius_factor, RadiusFlag flag,
                                                                   const bool check_proj_error,
                                                                   const double dist_th,
                                                                   const bool use_ratio_test, const float ratio)
{
  // Wrap matcher parameters
  cv::Mat query_descriptors;
  std::vector<size_t> query_indices;
  std::vector<std::vector<size_t>> guided_train_indices;

  for (size_t i = 0; i < query_mappoints.size(); ++i) {
    std::shared_ptr<MapPoint> mp = query_mappoints[i];

    cv::Mat uv;
    int octave;
    double view_cosine;
    cv::Mat range = cv::Mat(2, 2, CV_64F);
    double radius = 0;
    if (mp && mp->IsProjectable(train_frame, uv, octave, view_cosine)) {
      switch (flag) {
        case kRadiusFromViewCosine: {
          // TUNE: 
          if (view_cosine > 0.998) {
            radius = 2.5;
          } else {
            radius = 4.0;
          }
          break;
        }
        case kRadiusFromOctave: {
          radius = scale_factors_[octave];
          break;
        }
        default: {
          CHECK(false) << "invalid RadiusFlag.";
        }
      }

      radius = radius * radius_factor;

      // TODO: wrap PointsInRange(double x, double y, double radius);
      double x = uv.at<double>(0);
      double y = uv.at<double>(1);
      range.at<double>(0, 0) = x - radius;
      range.at<double>(0, 1) = y - radius;
      range.at<double>(1, 0) = x + radius;
      range.at<double>(1, 1) = y + radius;
      std::vector<size_t> indices = train_frame->range_searcher()->PointsInRange(range);
      // Check octave, we only keep the keypoint index with octave in [octave-1, octave]
      // Also skip the already associated mappoint? 
      // ORB SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th) does, we don't
      auto cull_indices_lambda = [&](size_t& i) -> bool { 
        const cv::KeyPoint& train_kp = train_frame->undistorted_kp(i);
        bool proj_error = false;
        if (check_proj_error) {
          const double d_x = x - train_kp.pt.x;
          const double d_y = y - train_kp.pt.y;
          const double error_th = 5.991 * level_sigma2_[train_kp.octave];
          if ((d_x*d_x + d_y*d_y) > error_th)
            proj_error = true;
        }
        // TODO:!train_frame->mappoint(i)
        return train_kp.octave < octave - 1 || train_kp.octave > octave || proj_error;// || train_frame->mappoint(i); 
      };

      indices.erase(std::remove_if(indices.begin(), indices.end(), cull_indices_lambda),
                    indices.end());

      guided_train_indices.push_back(indices);
      query_descriptors.push_back(mp->descriptors());
      query_indices.push_back(i);
    }
  }

  // We search
  std::vector<cv::DMatch> matches = Matcher(query_descriptors, query_indices,
                                    train_frame->descriptors(), guided_train_indices,
                                    dist_th, use_ratio_test, ratio);

  return matches;
}

// TODO: not pure 2d2d matcher, assuming associated mappoint not null
std::vector<cv::DMatch> GuidedMatcher::DbowGuided2D2DMatcher(std::shared_ptr<Frame> query_frame,
                                                             std::shared_ptr<Frame> train_frame,
                                                             const double dist_th,
                                                             const bool check_rotation,
                                                             const bool use_ratio_test, const float ratio)
{

  // Wrap matcher parameters
  cv::Mat query_descriptors;
  std::vector<size_t> query_indices;
  std::vector<std::vector<size_t>> guided_train_indices;

  cv::Mat query_frame_descriptors = query_frame->descriptors();

  // Compute DBow
  query_frame->ComputeBoW();
  train_frame->ComputeBoW();
  // DBoW guided
  auto query_feat_vec = query_frame->feature_vector();
  auto train_feat_vet = train_frame->feature_vector();
  auto query_it = query_feat_vec.begin();
  auto query_end = query_feat_vec.end();
  auto train_it = train_feat_vet.begin();
  auto train_end = train_feat_vet.end();

  while (query_it != query_end && train_it != train_end)
  {
    if (query_it->first == train_it->first)
    {
      const std::vector<size_t> node_query_indices(query_it->second.begin(), query_it->second.end());
      const std::vector<size_t> node_train_indices(train_it->second.begin(), train_it->second.end());
      for (auto &query_index : node_query_indices)
      {
        // Skip the descriptor with no mappoint associated
        auto query_mappoint = query_frame->mappoint(query_index);
        if (!query_mappoint || query_mappoint->is_bad())
          continue;

        query_descriptors.push_back(query_frame_descriptors.row(query_index));
        query_indices.push_back(query_index);
        guided_train_indices.push_back(node_train_indices);
      }

      query_it++;
      train_it++;
    }
    else if (query_it->first < train_it->first)
    {
      query_it = query_feat_vec.lower_bound(train_it->first);
    }
    else
    {
      train_it = train_feat_vet.lower_bound(query_it->first);
    }
  }

  // We perform guided match
  auto matches = Matcher(query_descriptors, query_indices,
                         train_frame->descriptors(), guided_train_indices,
                         dist_th, use_ratio_test, ratio);

  if (check_rotation)
    matches = CheckRotation(query_frame, train_frame, matches);

  return matches;
}

std::vector<cv::DMatch> GuidedMatcher::DbowAndEpipolarGuided2D2DMatcher(std::shared_ptr<Frame> query_frame, 
                                                          std::shared_ptr<Frame> train_frame, 
                                                          const double dist_th, 
                                                          const bool check_rotation,
                                                          const bool use_ratio_test, const float ratio) {

  auto matches = DbowGuided2D2DMatcher(query_frame, train_frame, dist_th, check_rotation, use_ratio_test, ratio);
  // Recover Fundamental Matrix according to poses
  cv::Mat F = RecoverFundamental(query_frame, train_frame);
  // Compute epipole in second image
  cv::Mat queryframe_o_w = query_frame->o_w();
  cv::Mat qf_o_trainframe = train_frame->Project(queryframe_o_w);
  cv::Mat uv = train_frame->camera_model()->Project(qf_o_trainframe);
  double ex = uv.at<double>(0);
  double ey = uv.at<double>(1);

  auto cull_match_lambda = [&](cv::DMatch& match) -> bool {
    // Skip the match which keypoint have a corresponding mappoint
    if (query_frame->mappoint(match.queryIdx) || train_frame->mappoint(match.trainIdx))
      return true;
    
    const cv::KeyPoint unkp_query = query_frame->undistorted_kp(match.queryIdx);
    const cv::KeyPoint unkp_train = train_frame->undistorted_kp(match.trainIdx);

    // The mappoint is too close to query_frame
    const double delta_x = ex - unkp_train.pt.x;
    const double delta_y = ey - unkp_train.pt.y;
    // TUNE:
    if (delta_x*delta_x + delta_y*delta_y < 100*scale_factors_[unkp_train.octave])
      return true;
    
    // Check epipolar constraint
    // TUNE:
    double th = 3.84*level_sigma2_[unkp_train.octave];
    if (!CheckDistEpipolarLine(unkp_query, unkp_train, F, th))
      return true;

    return false;
  };
  
  // Epipolar constraint
  matches.erase(std::remove_if(matches.begin(), matches.end(), cull_match_lambda), 
                matches.end());

  return matches;
}



// TODO: set all member functions as static function
std::vector<cv::DMatch> GuidedMatcher::CheckRotation(std::shared_ptr<Frame> query_frame,
                                                     std::shared_ptr<Frame> train_frame,
                                                     std::vector<cv::DMatch> matches)
{

  // cv::KeyPoint::angle:
  //    computed orientation of the keypoint (-1 if not applicable);
  //    it's in [0,360) degrees and measured relative to image coordinate system, ie in clockwise.
  // The rotation of two coherent keypoint is in (-360,360) degrees,
  // We construct a rot_hist to classify the rotation degrees, and pick up the most common three histogram
  std::vector<cv::DMatch> final_matches;
  // TUNE: 30
  int histo_len = 30;
  std::vector<std::vector<int>> rot_hist(histo_len, std::vector<int>());
  const float histo_width = 360.0f / histo_len;
  // Construct a rotation histogram from matches
  for (int idx = 0; idx < matches.size(); ++idx)
  {
    cv::DMatch match = matches[idx];
    float rot = query_frame->undistorted_kp(match.queryIdx).angle -
                train_frame->undistorted_kp(match.trainIdx).angle;
    if (rot < 0.0)
      rot += 360.0f;
    int bin = std::floor(rot / histo_width);
    CHECK(bin >= 0 && bin < histo_len);
    rot_hist[bin].push_back(idx);
  }

  int ind1 = -1, ind2 = -1, ind3 = -1;
  ComputeThreeMaxima(rot_hist, histo_len, ind1, ind2, ind3);
  // We only accept the match that fall into the first three strongest columns
  for (int i = 0; i < histo_len; ++i)
  {
    if (i == ind1 || i == ind2 || i == ind3)
    {
      for (size_t j = 0; j < rot_hist[i].size(); ++j)
      {
        final_matches.push_back(matches[rot_hist[i][j]]);
      }
    }
  }

  return final_matches;
}

int GuidedMatcher::DescriptorDist(const cv::Mat &a, const cv::Mat &b)
{
  // Copied from ORB-SLAM2
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++)
  {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

void GuidedMatcher::ComputeThreeMaxima(const std::vector<std::vector<int>> &histo, const int L, int &ind1, int &ind2, int &ind3)
{
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++)
  {
    const int s = histo[i].size();
    if (s > max1)
    {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    }
    else if (s > max2)
    {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    }
    else if (s > max3)
    {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1)
  {
    ind2 = -1;
    ind3 = -1;
  }
  else if (max3 < 0.1f * (float)max1)
  {
    ind3 = -1;
  }
}

} // namespace lslam
