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
#include "range_searcher.h"

namespace lslam {
class MapPoint;

class Frame{ 
public:
  
  // No default constructor
  Frame();
  
  Frame(double timestamp, unsigned long id, const cv::Mat& image);
  // Copy constructor
  Frame(const Frame& frame);
  Frame& operator=(const Frame&) = delete;
  
  ~Frame() {
  }
  
  // Function to be called after construction
  // We pick the function out of construction in case we need a specific thread to perform preprocess
  // 1. Extract ORB feature; 
  // 2. Undistortd keypoints and assign them to grid for fast range searching
  // 3. Allocating 
  void PreProcess(std::shared_ptr<ORB_SLAM2::ORBextractor>,std::shared_ptr<PinholeCamera>);

  // Accessors
  virtual unsigned long id() const;
  cv::Mat image() const;
  std::vector<cv::KeyPoint> keypoints() const;
  const std::vector<cv::KeyPoint>& undistorted_kps() const;
  const cv::KeyPoint& undistorted_kp(size_t idx) const;
  const cv::Mat& descriptors() const;
  std::vector<std::shared_ptr<MapPoint>> mappoints() const;
  std::shared_ptr<RangeSearcher> range_searcher() const;
  std::shared_ptr<MapPoint> mappoint(size_t idx) const; 
  std::shared_ptr<PinholeCamera> camera_model() const;
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor() const;
  bool outlier(size_t) const;

  void SetPose(cv::Mat T_cw);
  void set_mappoint(size_t idx, std::shared_ptr<MapPoint> mappoint);
  void set_T_cl(cv::Mat T_cl);
  void set_outlier(size_t idx, bool flag);

  cv::Mat T_cw() const;
  cv::Mat T_wc() const;
  cv::Mat T_cl() const;

  // Project world coordinate 3d point to image coordinate 3d point
  cv::Mat Project(const cv::Mat pt3d_w);
  
protected:
  // === Raw data ===
  double timestamp_; // timestamp of this frame
  unsigned long id_; // id of this frame
  cv::Mat image_; // the image as OpenCV's matrix
  // === Raw data ===

  // The camera geometry
  std::shared_ptr<PinholeCamera> camera_model_;
  // Feature detector and descriptor extractor
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;

  // === Data evidence ===
  std::vector<cv::KeyPoint> keypoints_; // the keypoints as OpenCV's struct
  std::vector<cv::KeyPoint> undistorted_kps_; // The undistorted keypoints
  cv::Mat descriptors_; // the descriptors as OpenCV's matrix 
  // === Data evidence ===

  // === State ===
  // MapPoints associated to keypoints, NULL for no association
  std::vector<std::shared_ptr<MapPoint>> mappoints_;
  // Flag to identify outlier associations
  std::vector<bool> outliers_; 
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
