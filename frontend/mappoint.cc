/*
 * Author: ShiQin
 * Date: 2017-08-29
 */

#include "mappoint.h"

#include "frame.h"
#include "keyframe.h"
#include <limits>

namespace lslam {

// MapPoint::MapPoint() 
//   : is_bad_(false) {
//   static unsigned long unique_id = 0;
//   id_ = unique_id++;
// }

MapPoint::MapPoint(const cv::Mat& pt_world)
  : is_bad_(false)
  , pt_world_(pt_world) {

  static unsigned long unique_id = 0;
  id_ = unique_id++;
}

void MapPoint::AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index) {
  // If this keyframe already exists, we insert nothing, else insert
  observations_.insert({keyframe, keypoint_index});
}

size_t MapPoint::ObservationCount() const {
  return observations_.size();
}

void MapPoint::ComputeDistinctiveDescriptors() {
  
  size_t size = ObservationCount();
  
  // We retrieve all observed descriptors
  if (size == 0) return;
  std::vector<cv::Mat> all_descriptors;
  all_descriptors.reserve(size);
  for (auto& item:observations_) {
    all_descriptors.push_back(item.first->descriptors().row(item.second));
  }
  
  // Compute distances between them
  double distances[size][size];
  for (size_t i = 0; i < size; ++i) {
    distances[i][i] = 0;
    for (size_t j = i+1; j < size; ++j) {
      int dist_ij = cv::norm(all_descriptors[i], all_descriptors[j], cv::NORM_HAMMING);
      distances[i][j] = dist_ij;
      distances[j][i] = dist_ij;
    }
  }
  
  // We take the descriptor with least median distance to the rest
  double best_median = std::numeric_limits<double>::max();
  int best_idx = 0;
  for (size_t i = 0; i < size; ++i) {
    // For descriptor i, we get all the distances to other descriptors
    std::vector<double> dists(distances[i], distances[i] + size);
    std::sort(dists.begin(), dists.end());
    double median = dists[0.5*(size-1)];
    if (median < best_median) {
      best_median = median;
      best_idx = i;
    }
  }
  
  descriptors_ = all_descriptors[best_idx].clone();
}

void MapPoint::UpdateNormalAndDepth() {
  
}

bool MapPoint::IsProjectable(std::shared_ptr<Frame> frame, std::shared_ptr<PinholeCamera> camera_model, cv::Mat& p_uv) {
  CHECK(frame) << "frame is Null.";
  CHECK(camera_model) << "camera_model is Null.";

  if (!frame->T_cw().data) {
    LOG(ERROR) << "The pose of frame is NOT set.";
    return false;
  }
  // 3D Landmark point in camera coordinates
  cv::Mat p_c = frame->Project(pt_world_);
  
  // Check positive depth
  const double pz_c = p_c.at<double>(2);
  if (pz_c <= 0.0) 
    return false;

  // Project to image coordinate and check it is inside
  p_uv = camera_model->Project(p_c);
  double u = p_uv.at<double>(0);
  double v = p_uv.at<double>(1);
  cv::Mat img_bounds = camera_model->image_bounds();
  double min_u = img_bounds.at<double>(0);
  double min_v = img_bounds.at<double>(1);
  double max_u = img_bounds.at<double>(2);
  double max_v = img_bounds.at<double>(3);
  if (u < min_u || u > max_u || v < min_v || v > max_v)
    return false;

  return true;
  
  
}

void MapPoint::set_pt_world(const cv::Mat& pt_world) {
  CHECK(pt_world.cols == 1 && pt_world.rows == 3) << "invalid pt_world dimension.";
  pt_world.copyTo(pt_world_);
}

cv::Mat MapPoint::pt_world() const {
  return pt_world_.clone();
}

cv::Mat MapPoint::descriptors() const {
  return descriptors_;
}

std::map<std::shared_ptr<KeyFrame>, size_t> MapPoint::observations() const {
  return observations_;
}

unsigned long MapPoint::id() const {
  return id_;
}

bool MapPoint::is_bad() const {
  return is_bad_;
}

} // namespace lslam
