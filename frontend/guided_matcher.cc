/*
 * @Author: Shi Qin 
 * @Date: 2017-09-25 15:19:52 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 21:01:20
 */

#include "guided_matcher.h"

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
          cur_frame->AddLandmark(landmark, idx_bestmatch);
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
          cur_frame->AddLandmark(static_cast<std::shared_ptr<Landmark>>(NULL), rot_hist[i][j]);
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