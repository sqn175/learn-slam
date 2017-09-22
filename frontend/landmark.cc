/*
 * Author: ShiQin
 * Date: 2017-08-29
 */

#include <limits>

#include "landmark.h"

namespace lslam {

Landmark::Landmark() {
}

Landmark::Landmark(Eigen::Vector4d point_world) 
  : id_(0), point_world_(point_world) { }

void Landmark::AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index) {
  // If this keyframe already exists, we insert nothing, else insert
  observations_.insert({keyframe, keypoint_index});
}

size_t Landmark::ObservationCount() const {
  return observations_.size();
}

void Landmark::ComputeDistinctiveDescriptors() {
  
  size_t size = ObservationCount();
  
  // We retrieve all observed descriptors
  if (size == 0) return;
  std::vector<cv::Mat> all_descriptors;
  all_descriptors.reserve(size);
  for (auto i = observations_.begin(); i != observations_.end(); ++i) {
    std::shared_ptr<CameraMeasurement> cm_i = (*i).first->camera_measurement();
    all_descriptors.push_back(cm_i->descriptors().row((*i).second));
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

cv::Mat Landmark::point_world() const {
  cv::Mat point_world(4,1,CV_64FC1);
  cv::eigen2cv(point_world_, point_world);
  return point_world;
}

} // namespace lslam
