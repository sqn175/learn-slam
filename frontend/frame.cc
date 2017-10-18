/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include "frame.h"

#include <iostream>
#include "my_assert.h"
#include "mappoint.h"

namespace lslam {

Frame::Frame() {
}

Frame::Frame(double timestamp, unsigned long id, const cv::Mat& image)
  : timestamp_(timestamp)
  , id_(id) 
  , image_(image.clone()) {
}

Frame::Frame(const Frame& frame) 
  : timestamp_(frame.timestamp_), id_(frame.id_), image_(frame.image_.clone())
  , camera_model_(frame.camera_model_), orb_extractor_(frame.orb_extractor_)
  , keypoints_(frame.keypoints_), undistorted_kps_(frame.undistorted_kps_), descriptors_(frame.descriptors_.clone())
  , mappoints_(frame.mappoints_), outliers_(frame.outliers_)
  , T_cw_(frame.T_cw_.clone()), R_cw_(frame.R_cw_.clone()), t_cw_(frame.t_cw_.clone()), o_w_(frame.o_w_.clone())
  , T_wc_(frame.T_wc_.clone()), T_cl_(frame.T_cl_.clone())
  , range_searcher_(frame.range_searcher_) {
}

void Frame::PreProcess(std::shared_ptr<ORB_SLAM2::ORBextractor> extractor,std::shared_ptr<PinholeCamera> camera_model) {
  orb_extractor_ = extractor;
  camera_model_ = camera_model;
  
  // Extract ORB
  CHECK(image_.data) << "This Frame is not initialized with image";
  (*extractor)(image_, cv::Mat(), keypoints_, descriptors_);
  // Undistort
  if (camera_model->DistortionType().compare("radialtangential") == 0 ) {
    cv::Mat mat(keypoints_.size(), 2, CV_32F);
    for (int i = 0 ; i < keypoints_.size(); ++i) {
      mat.at<float>(i,0) = keypoints_[i].pt.x;
      mat.at<float>(i,1) = keypoints_[i].pt.y;
    }
    // Undistort keypoints using OpenCV function
    mat = mat.reshape(2);

    cv::undistortPoints(mat, mat, camera_model->K(), camera_model->DistCoeffs(), cv::noArray(), camera_model->K());
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    undistorted_kps_.resize(keypoints_.size());
    for (int i = 0; i < keypoints_.size(); ++i) {
      cv::KeyPoint kp = keypoints_[i];
      kp.pt.x = mat.at<float>(i,0);
      kp.pt.y = mat.at<float>(i,1);
      undistorted_kps_[i] = kp;
    }

  } else {
    undistorted_kps_ = keypoints_;
  }

  // SetPose
  SetPose(cv::Mat::eye(4,4,CV_64F));

  // Allocate mappoints of NULL
  mappoints_ = std::vector<std::shared_ptr<MapPoint>>(keypoints_.size(), static_cast<std::shared_ptr<MapPoint>>(NULL));

  // Initialize outlier flag
  outliers_ = std::vector<bool>(mappoints_.size(), false);

  // Build gridding range searcher
  range_searcher_ = std::make_shared<RangeSearcher>(undistorted_kps_, camera_model->image_bounds(), 1);
  
}

unsigned long Frame::id() const {
  return id_;
}
cv::Mat Frame::image() const {
  return image_;
}

std::vector<cv::KeyPoint> Frame::keypoints() const {
  return keypoints_;
}

const std::vector<cv::KeyPoint>& Frame::undistorted_kps() const {
  return undistorted_kps_;
}

const cv::Mat& Frame::descriptors() const {
  return descriptors_;
}

const cv::KeyPoint& Frame::undistorted_kp(size_t idx) const {
  return undistorted_kps_[idx];
}

std::vector<std::shared_ptr<MapPoint>> Frame::mappoints() const {
  return mappoints_;
}

std::shared_ptr<RangeSearcher> Frame::range_searcher() const {
  return range_searcher_;
}

std::shared_ptr<PinholeCamera> Frame::camera_model() const {
  return camera_model_;
}

std::shared_ptr<ORB_SLAM2::ORBextractor> Frame::orb_extractor() const {
  return orb_extractor_;
}

bool Frame::outlier(size_t idx) const {
  return outliers_[idx];
}

void Frame::SetPose(cv::Mat T_cw) {
  T_cw_ = T_cw.clone();
  R_cw_ = T_cw.rowRange(0,3).colRange(0,3);
  t_cw_ = T_cw.rowRange(0,3).col(3);
  o_w_ = -R_cw_.t() * t_cw_;
  
  T_wc_ = cv::Mat::eye(4,4,T_cw_.type());
  cv::Mat R_wc_ = R_cw_.t();
  R_wc_.copyTo(T_wc_.rowRange(0,3).colRange(0,3));
  o_w_.copyTo(T_wc_.rowRange(0,3).col(3));
}

void Frame::set_T_cl(cv::Mat T_cl) {
  T_cl_ = T_cl.clone();
}

void Frame::set_outlier(size_t idx, bool flag) {
  outliers_[idx] = flag;
}


void Frame::set_mappoint( size_t idx, std::shared_ptr<MapPoint> landmark) {
  mappoints_[idx] = landmark;
}

std::shared_ptr<MapPoint> Frame::mappoint(size_t idx) const {
  return mappoints_[idx];
}

// TODO: consider if we need deep clone?
cv::Mat Frame::T_cw() const {
  return T_cw_.clone();
}

cv::Mat Frame::T_wc() const {
  return T_wc_.clone();
}

cv::Mat Frame::T_cl() const {
  return T_cl_.clone();
}

cv::Mat Frame::Project(const cv::Mat pt3d_w) {
  CHECK(pt3d_w.cols == 1 && pt3d_w.rows == 3) << "Invalid 3d point.";
  CHECK(T_cw_.data) << "The pose of frame is NOT set.";
  cv::Mat p_c = R_cw_*pt3d_w + t_cw_;
  return p_c;
}

} // namespace lslam
