/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_CAMERA_MEASUREMENT_H_
#define FRONTEND_CAMERA_MEASUREMENT_H_

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ORBextractor.h>
#include "pinhole_camera.h"
#include "landmark.h"
#include "range_searcher.h"

namespace lslam {

class Landmark;

class Frame{ 
public:
  
  // No default constructor
  Frame();
  
  Frame(unsigned long timestamp, unsigned long id, const cv::Mat& image);
  
  ~Frame() {
  }
  
  // Function to be called after construction
  // 1. Extract ORB feature; 
  // 2. Undistort keypoints and assign them to grid for fast range searching
  // 3. Allocating 
  void PreProcess(std::shared_ptr<ORB_SLAM2::ORBextractor>,std::shared_ptr<PinholeCamera>);


  // Accessors
  cv::Mat image() const;
  std::vector<cv::KeyPoint> keypoints() const;
  cv::Mat descriptors() const;
  std::vector<std::shared_ptr<Landmark>> landmarks() const;
  std::shared_ptr<RangeSearcher> range_searcher() const;

  void SetPose(cv::Mat T_cw);
  void AddLandmark(std::shared_ptr<Landmark> landmark, size_t idx);
  void set_T_cl(cv::Mat T_cl);
  
  cv::Mat T_cw() const;
  cv::Mat T_wc() const;
  cv::Mat T_cl() const;

  // Project world coordinate 3d point to image coordinate 3d point
  cv::Mat Project(const cv::Mat pt3d_w);
  
private:
  // === Raw data ===
  double timestamp_; // timestamp of this frame
  unsigned long id_; // id of this frame
  cv::Mat image_; // the image as OpenCV's matrix
  // === Raw data ===

  // === Data evidence ===
  std::vector<cv::KeyPoint> keypoints_; // the keypoints as OpenCV's struct
  std::vector<cv::KeyPoint> undistorted_kps_; // The undistorted keypoints
  cv::Mat descriptors_; // the descriptors as OpenCV's matrix 
  // === Data evidence ===

  // === State ===
  // Landmarks associated to keypoints, NULL for no association
  std::vector<std::shared_ptr<Landmark>> landmarks_;
  // Camera Pose, cw -> world coordinate to camera coordinate
  cv::Mat T_cw_; // transformation matrix
  cv::Mat R_cw_; // rotation matrix
  cv::Mat t_cw_; // translation vector
  cv::Mat o_w_;  // camera original in world coordinate
  // Camera inverse pose, wc -> camera coordinate to world coordinate
  cv::Mat T_wc_; // inverse transformation matrix
  // Transformation from last frame to current frame
  cv::Mat T_cl_;
  // === State ===

  // Build feature points gridding range searcher
  std::shared_ptr<RangeSearcher> range_searcher_;

};

} // namespace lslam

#endif // FRONTEND_CAMERA_MEASUREMENT_H_
