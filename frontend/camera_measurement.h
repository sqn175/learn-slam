/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_CAMERA_MEASUREMENT_H_
#define FRONTEND_CAMERA_MEASUREMENT_H_

#include <vector>
#include <memory>

#include <opencv2/core.hpp>

#include <ORBextractor.h>
#include "pinhole_camera.h"

namespace lslam {

class CameraMeasurement{ 
public:
  
  // No default constructor
  CameraMeasurement() {}
  
  CameraMeasurement(unsigned long id, const cv::Mat& image,
        std::shared_ptr<const PinholeCamera>& camera_model,
        std::shared_ptr<ORB_SLAM2::ORBextractor>& orb_extractor);
  
  CameraMeasurement(const CameraMeasurement&);
  
  ~CameraMeasurement() {
  }
  
  // Accessors
  std::vector<cv::KeyPoint> keypoints() const;
  cv::Mat descriptors() const;
  std::shared_ptr<const PinholeCamera> camera_model() const;
  
  // Detect ORB keypoints and extract descriptors
  void ExtractORB();
  
  void SetPose(cv::Mat T_cw);
  
private:

  
  unsigned long id_; // id of this frame
  cv::Mat image_; // the image as OpenCV's matrix
  std::shared_ptr<const PinholeCamera> camera_model_;
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_; // feature detector and descriptor using ORBSLAM modules
  
  std::vector<cv::KeyPoint> keypoints_; // the keypoints as OpenCV's struct
  cv::Mat descriptors_; // the descriptors as OpenCV's matrix 
  
  // Camera Pose, cw -> world coordinate to camera coordinate
  cv::Mat T_cw_; // transformation matrix
  cv::Mat R_cw_; // rotation matrix
  cv::Mat t_cw_; // translation vector
  cv::Mat o_w_;  // camera original in world coordinate
};

} // namespace lslam

#endif // FRONTEND_CAMERA_MEASUREMENT_H_
