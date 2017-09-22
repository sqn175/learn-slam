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
  
  CameraMeasurement(unsigned long timestamp, unsigned long id, const cv::Mat& image);
  
  CameraMeasurement(const CameraMeasurement&);
  
  ~CameraMeasurement() {
  }
  
  void ExtractOrb(std::shared_ptr<ORB_SLAM2::ORBextractor>);
  
  // Accessors
  std::vector<cv::KeyPoint> keypoints() const;
  cv::Mat descriptors() const;
  
  void SetPose(cv::Mat T_cw);
  
  cv::Mat Tcw() const;
  cv::Mat Twc() const;
  
private:

  unsigned long timestamp_; // timestamp of this frame
  unsigned long id_; // id of this frame
  cv::Mat image_; // the image as OpenCV's matrix

  std::vector<cv::KeyPoint> keypoints_; // the keypoints as OpenCV's struct
  cv::Mat descriptors_; // the descriptors as OpenCV's matrix 
  
  // Camera Pose, cw -> world coordinate to camera coordinate
  cv::Mat T_cw_; // transformation matrix
  cv::Mat R_cw_; // rotation matrix
  cv::Mat t_cw_; // translation vector
  cv::Mat o_w_;  // camera original in world coordinate
  // Camera inverse pose, wc -> camera coordinate to world coordinate
  cv::Mat T_wc_; // inverse transformation matrix
};

} // namespace lslam

#endif // FRONTEND_CAMERA_MEASUREMENT_H_
