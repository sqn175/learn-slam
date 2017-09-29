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

#include "keyframe.h"
#include "frame.h"
#include "pinhole_camera.h"

namespace lslam {

class Frame;
class KeyFrame;

// TODO: thread safe???
class Landmark {
public:
  // Constructor
  Landmark();
  ~Landmark() { }
  
  void AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index);
  
  // Return the observation numbers
  size_t ObservationCount() const;
  
  // Compute the best descriptor associated to this landmark
  void ComputeDistinctiveDescriptors();

  // Check if this landmark is in the frustum of the camera
  bool IsProjectable(std::shared_ptr<Frame> frame, std::shared_ptr<PinholeCamera> camera_model, cv::Mat& uv);
  
  // Mutators
  void set_pt_world(const cv::Mat&);
  // Accessors
  cv::Mat pt_world() const;
  cv::Mat descriptors() const;
  
private:
  unsigned int id_; // ID of the landmark point
  cv::Mat pt_world_; // 3d coordinate of the landmark point in the world map
  cv::Mat descriptors_; // Best descriptors of this point
  // Keyframes observing the point and the associated keypoint index in keyframe
  std::map<std::shared_ptr<KeyFrame>, int> observations_; 
};

} // namespace lslam

#endif // FRONTEND_LANDMARK_H
