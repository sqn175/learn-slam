/*
 * Author: ShiQin
 * Date: 2017-08-24
 */

#ifndef FRONTEND_LANDMARK_H
#define FRONTEND_LANDMARK_H

#include <memory>
#include <map>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "keyframe.h"

namespace lslam {

class Landmark {
public:
  // Constructor
  Landmark();
  Landmark(Eigen::Vector4d point_world);
  ~Landmark() { }
  
  void AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index);
  
  // Return the observation numbers
  size_t ObservationCount() const;
  
  // Compute the best descriptor associated to this landmark
  void ComputeDistinctiveDescriptors();
  
  cv::Mat point_world() const;
  
private:
  unsigned int id_; // ID of the landmark point
  Eigen::Vector4d point_world_; // Homogenuous coordinate of the landmark point in the world map
  cv::Mat descriptors_; // Best descriptors of this point
  // Keyframes observing the point and the associated keypoint index in keyframe
  std::map<std::shared_ptr<KeyFrame>, int> observations_; 
};

} // namespace lslam

#endif // FRONTEND_LANDMARK_H
