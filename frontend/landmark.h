/*
 * Author: ShiQin
 * Date: 2017-08-24
 */

#ifndef FRONTEND_LANDMARK_H
#define FRONTEND_LANDMARK_H

#include <memory>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "keyframe.h"

namespace lslam {

class Landmark {
public:
  // Constructor
  Landmark();
  Landmark(unsigned int id, Eigen::Vector4d point_world);
  ~Landmark() { }
  
  // Compute the descriptors associated to this landmark
  void ComputeDistinctiveDescriptors();
  
private:
  unsigned int id_; // ID of the landmark point
  Eigen::Vector4d point_world_; // Homogenuous coordinate of the landmark point in the world map
  cv::Mat descriptors_; // Best descriptors of this point
  std::vector<std::shared_ptr<KeyFrame>> observations_; // Keyframe index vector observing the landmark point
};

} // namespace lslam

#endif // FRONTEND_LANDMARK_H
