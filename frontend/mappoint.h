/*
 * Author: ShiQin
 * Date: 2017-08-24
 */

#ifndef FRONTEND_LANDMARK_H
#define FRONTEND_LANDMARK_H

#include <memory>
#include <map>

#include <glog/logging.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "pinhole_camera.h"

namespace lslam {

class Frame;
class KeyFrame;

// TODO: thread safe???
class MapPoint {
public:
  // Constructor
  // MapPoint();
  MapPoint(const cv::Mat& pt_world, std::shared_ptr<KeyFrame> ref_kf);
  ~MapPoint() { }
  
  void AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index);
  
  // Return the observation numbers
  size_t ObservationCount() const;
  
  // Compute the best descriptor associated to this landmark
  void ComputeDistinctiveDescriptors();

  // Called after setting pt_world or reference_keyframe's o_w_
  void UpdateNormalAndDepth();

  int PredictOctaveInFrame(std::shared_ptr<Frame> frame, const double& dist);
  // Check if this map is in the frustum of the camera
  bool IsProjectable(std::shared_ptr<Frame> frame, cv::Mat& uv, int& scale, double& view_cosine);
  
  // Mutators
  void set_pt_world(const cv::Mat&);
  // Accessors
  unsigned long id() const;
  cv::Mat pt_world() const;
  cv::Mat descriptors() const;
  std::map<std::shared_ptr<KeyFrame>, size_t> observations() const;
  bool is_bad() const;
  
private:
  unsigned long id_; // ID of the map point
  cv::Mat pt_world_; // 3d coordinate of the map point in the world map
  cv::Mat descriptors_; // Best descriptors of this point
  // Keyframes observing the point and the associated keypoint index in keyframe
  std::map<std::shared_ptr<KeyFrame>, size_t> observations_; 
  cv::Mat normal_vector_; // Viewing direction

  // Reference keyframe, from which the mappoint is created(by triangulation)
  std::shared_ptr<KeyFrame> ref_keyframe_;
  // scale invariance distance
  double max_distance_; 
  double min_distance_;


  //
  bool is_bad_;
};

} // namespace lslam

#endif // FRONTEND_LANDMARK_H
