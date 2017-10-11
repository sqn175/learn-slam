/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:19:52 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 21:01:20
 */

#include "guided_matcher.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "limits"

namespace lslam {

const int GuidedMatcher::TH_HIGH = 100;
const int GuidedMatcher::TH_LOW = 50;
const int GuidedMatcher::HISTO_LENGTH = 30;

void GuidedMatcher::set_camera_model(std::shared_ptr<PinholeCamera> camera_model) {
  camera_model_ = camera_model;
}

void GuidedMatcher::set_orb_extractor( std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor) {
  orb_extractor_ = orb_extractor;
  scale_levels_ = orb_extractor->GetLevels();
  scale_factor_ = orb_extractor->GetScaleFactor();
  log_scale_factor_ = std::log(scale_factor_);
  scale_factors_ = orb_extractor->GetScaleFactors();
  inv_scale_factors_ = orb_extractor->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor->GetInverseScaleSigmaSquares();
}
  
int GuidedMatcher::Matcher(const cv::Mat& query_descriptors, const std::shared_ptr<Frame> train_frame, 
                            const std::vector<cv::Mat>& guided_search_ranges,
                            std::vector<cv::DMatch>& matches,
                            const float dist_th, const bool use_ratio_test, const float ratio) {

  // TODO: check input is valid
  int n_match = 0;
  int size = guided_search_ranges.size();
  CHECK(query_descriptors.rows == size);
  matches.clear();
  matches.reserve(size);
  matches = std::vector<cv::DMatch>(size, cv::DMatch(-1,-1,std::numeric_limits<float>::max()));

  cv::Mat train_descriptors = train_frame->descriptors();
  std::vector<cv::DMatch> inv_matches(train_descriptors.rows, cv::DMatch(-1,-1,std::numeric_limits<float>::max()));

  for (int i = 0; i < size; ++i) {
    // Zero mat ranges means its invalid
    if (cv::countNonZero(guided_search_ranges[i]) < 1) {
      matches.push_back(cv::DMatch(i,-1,std::numeric_limits<float>::max()));
      continue;
    }
    std::vector<size_t> indices = train_frame->range_searcher()->PointsInRange(guided_search_ranges[i]);
    
    if (indices.empty())
      continue;

    cv::Mat query_descriptor = query_descriptors.row(i);
    int train_idx = -1;

    // We search in indices
    float min_dist = std::numeric_limits<float>::max();
    float min_dist2 = std::numeric_limits<float>::max();

    for (auto& idx : indices) {
      float dist = DescriptorDist(query_descriptor, train_descriptors.row(idx));

      if (dist < min_dist) {
        if (use_ratio_test) {
          min_dist2 = min_dist;
        }
        min_dist = dist;
        train_idx = idx;
      } else if (use_ratio_test && dist < min_dist2) {
        min_dist2 = dist;
      }
    }

    // If the distance is above the dist_th, the match is invalid
    if (min_dist > dist_th) {
      matches[i] = cv::DMatch(i,-1,std::numeric_limits<float>::max());
      continue;
    }

    if (use_ratio_test) {
      // We apply the ratio test
      if (min_dist >= ratio * min_dist2) {
        matches[i] = cv::DMatch(i,-1,std::numeric_limits<float>::max());
        continue;
      }
    }

    // We get a match, but we need it to be one to one 
    cv::DMatch inv_match = inv_matches[train_idx];
    if (inv_match.trainIdx >= 0) {
      if (inv_match.distance < min_dist) {
        matches[i] = cv::DMatch(i,-1,std::numeric_limits<float>::max());
        continue;
      }
    }
    matches[i] = cv::DMatch(i, train_idx, min_dist);
    inv_matches[i] = cv::DMatch(train_idx, i, min_dist);
    n_match++;
  }

  return n_match;
}

void GuidedMatcher::SetupGuided2D2DMatcher(std::shared_ptr<Frame> init_frame) {
  init_query_frame_ = init_frame;
  const std::vector<cv::KeyPoint>& undistorted_kps = init_frame->undistorted_kps();
  guided_2d_pts_.resize(undistorted_kps.size());
  for (int i = 0; i < undistorted_kps.size(); ++i) {
    guided_2d_pts_[i] = undistorted_kps[i].pt;
  }
  init_query_descriptors_ = init_frame->descriptors();
  std::cout<<init_query_descriptors_.rowRange(0,10)<<std::endl;
}

int GuidedMatcher::Guided2D2DMatcher(std::shared_ptr<Frame> train_frame, std::vector<cv::DMatch> matches, 
                                      const int radius, const float dist_th,
                                      const bool use_ratio_test, const float ratio, const bool check_rotation ) {
  // Wrap guided_search_ranges, zero mat range means its invalid 
  std::vector<cv::Mat> search_ranges(guided_2d_pts_.size(), cv::Mat::zeros(2,2,CV_64F));
  for (int i = 0; i < guided_2d_pts_.size(); ++i) {
    // We only search octave 0
    if (init_query_frame_->undistorted_kp(i).octave > 0)
      continue;
    search_ranges[i].at<double>(0,0) = (double)guided_2d_pts_[i].x - radius;
    search_ranges[i].at<double>(0,1) = (double)guided_2d_pts_[i].y - radius;
    search_ranges[i].at<double>(1,0) = (double)guided_2d_pts_[i].x + radius;
    search_ranges[i].at<double>(1,1) = (double)guided_2d_pts_[i].y + radius;
  }

  // We search
  int n_match = Matcher(init_query_descriptors_, train_frame, search_ranges, matches, dist_th, use_ratio_test, ratio);

  cv::Mat img_matches;
  std::vector<cv::DMatch> match_test;

  // Check rotation
  if (check_rotation) {
    std::vector<std::vector<int>> rot_hist(HISTO_LENGTH, std::vector<int>());
    const float factor = HISTO_LENGTH / 360.0f;
    // Construc a rotation histogram from matches
    for (auto& match : matches) {
      if (match.trainIdx < 0)
        continue;
      float rot = init_query_frame_->undistorted_kp(match.queryIdx).angle - 
                  train_frame->undistorted_kp(match.trainIdx).angle;
      if (rot < 0.0)
        rot += 360.0f;
      int bin = std::round(rot*factor);
      if (bin == HISTO_LENGTH)
        bin = 0;
      CHECK(bin>=0 && bin<HISTO_LENGTH);
      rot_hist[bin].push_back(match.queryIdx);
    }

    int ind1 = -1, ind2 = -1, ind3 = -1;
    ComputeThreeMaxima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);
    // We only accept the match that fall into the first three strongest columns
    for (int i = 0; i < HISTO_LENGTH; ++i) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      // Kick out the match that do NOT pass rotation check
      for (size_t j = 0; j < rot_hist[i].size(); ++j) {
        int query_idx = rot_hist[i][j];
        matches[query_idx] = cv::DMatch(query_idx,-1,std::numeric_limits<float>::max());
        n_match--;
      }
    }
  }

  // Update guided_2d_pts
  for (size_t i = 0; i < matches.size(); ++i) {
    if (matches[i].trainIdx < 0)
      continue;
    guided_2d_pts_[i] = train_frame->undistorted_kp(matches[i].trainIdx).pt;
  }

  return n_match;
}

int GuidedMatcher::ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::vector<std::shared_ptr<Landmark>> landmarks) {
  int n_matches = 0;

  for (auto& landmark : landmarks) {
    // Check if this landmark is projection valid
    
  }
  return n_matches;
}

int GuidedMatcher::ProjectionGuided3D2DMatcher(std::shared_ptr<Frame> cur_frame, std::shared_ptr<Frame> last_frame, const double th, const bool check_ori) {
  int n_matches = 0;

  // Rotation Histogram (to check rotation consistency)
  std::vector<std::vector<int>> rot_hist(HISTO_LENGTH, std::vector<int>());
  const float factor = HISTO_LENGTH/360.0f;

  std::vector<std::shared_ptr<Landmark>> landmarks = last_frame->landmarks();
  std::vector<cv::KeyPoint> last_keypoints = last_frame->keypoints();

  cv::Mat cur_descriptors = cur_frame->descriptors();
  std::vector<cv::KeyPoint> cur_keypoints = cur_frame->keypoints();

  for (int i = 0; i < landmarks.size(); ++i) {
    auto landmark = landmarks[i];
    if (landmark) {
      // TODO: check landmark outlier
      // Project landmark to current frame, projection result is uv
      cv::Mat p_uv;
      if (landmark->IsProjectable(cur_frame, camera_model_, p_uv)) {
        int last_octave = last_keypoints[i].octave;
        // Search in a window, size depends on scale
        double r = th*scale_factors_[last_octave];
        
        // We search
        double u = p_uv.at<double>(0);
        double v = p_uv.at<double>(1);
        cv::Mat search_bounds(2,2,CV_64F);
        search_bounds.at<double>(0,0) = u-r;
        search_bounds.at<double>(0,1) = v-r;
        search_bounds.at<double>(1,0) = u+r;
        search_bounds.at<double>(1,1) = v+r;

        std::vector<size_t> indices = cur_frame->range_searcher()->PointsInRange(search_bounds);
        if (indices.empty())  continue;

        int min_dist = 256;
        int idx_bestmatch = -1;

        // Descriptor of this landmark
        cv::Mat descriptors_landmark = landmark->descriptors();

        for (auto& idx : indices) {
          // We already have a landmark associated to this feature
          if (cur_frame->landmarks()[idx] && (cur_frame->landmarks()[idx])->ObservationCount() > 0)
            continue;

          int dist = DescriptorDist(descriptors_landmark, cur_descriptors.row(idx));
          
          if (dist < min_dist) {
            min_dist = dist;
            idx_bestmatch = idx;
          } 
        }

        if (min_dist <= TH_HIGH)
        {
          cur_frame->set_landmark(idx_bestmatch, landmark);
          n_matches++;

          // Check orientation
          if (check_ori) {
            float angle_diff = last_keypoints[i].angle - cur_keypoints[idx_bestmatch].angle;
            if (angle_diff < 0.0) {
              angle_diff += 360.0f;
            }
            int bin = std::round(angle_diff*factor);
            if (bin == HISTO_LENGTH)
              bin = 0;
            CHECK(bin >= 0 && bin < HISTO_LENGTH);
            rot_hist[bin].push_back(idx_bestmatch);
          }
        }
      }
    }
  }

  if (check_ori) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; ++i) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0; j < rot_hist[i].size(); ++j) {
          cur_frame->set_landmark(rot_hist[i][j], static_cast<std::shared_ptr<Landmark>>(NULL));
          n_matches--;
        }
      }
    }

  }

  return n_matches;
}

int GuidedMatcher::DescriptorDist(const cv::Mat& a, const cv::Mat& b) {
  // Copied from ORB-SLAM2
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist=0;

  for(int i=0; i<8; i++, pa++, pb++)
  {
      unsigned  int v = *pa ^ *pb;
      v = v - ((v >> 1) & 0x55555555);
      v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
      dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

void GuidedMatcher::ComputeThreeMaxima(const std::vector<std::vector<int>>& histo, const int L, int& ind1, int& ind2, int& ind3) {
  int max1=0;
  int max2=0;
  int max3=0;

  for(int i=0; i<L; i++)
  {
      const int s = histo[i].size();
      if(s>max1)
      {
          max3=max2;
          max2=max1;
          max1=s;
          ind3=ind2;
          ind2=ind1;
          ind1=i;
      }
      else if(s>max2)
      {
          max3=max2;
          max2=s;
          ind3=ind2;
          ind2=i;
      }
      else if(s>max3)
      {
          max3=s;
          ind3=i;
      }
  }

  if(max2<0.1f*(float)max1)
  {
      ind2=-1;
      ind3=-1;
  }
  else if(max3<0.1f*(float)max1)
  {
      ind3=-1;
  }
}

} // namespace lslam
