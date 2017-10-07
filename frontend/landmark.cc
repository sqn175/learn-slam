/*
 * Author: ShiQin
 * Date: 2017-08-29
 */

#include "landmark.h"
#include <iostream>
#include <limits>

namespace lslam {

Landmark::Landmark() {
}

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
    std::shared_ptr<Frame> cm_i = (*i).first->frame();
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

bool Landmark::IsProjectable(std::shared_ptr<Frame> frame, std::shared_ptr<PinholeCamera> camera_model, cv::Mat& p_uv) {
  CHECK(frame) << "frame is Null.";
  CHECK(camera_model) << "camera_model is Null.";
  std::cout<<"Current camera Pose: \n"<<frame->T_cw()<<std::endl;
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

void Landmark::set_pt_world(const cv::Mat& pt_world) {
  CHECK(pt_world.cols == 1 && pt_world.rows == 3) << "invalid pt_world dimension.";
  pt_world.copyTo(pt_world_);
}

cv::Mat Landmark::pt_world() const {
  return pt_world_.clone();
}

cv::Mat Landmark::descriptors() const {
  return descriptors_;
}

} // namespace lslam
