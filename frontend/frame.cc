/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#include <iostream>

#include "frame.h"

#include "my_assert.h"

namespace lslam {

Frame::Frame() {
  SetPose(cv::Mat::eye(4,4,CV_64F));
  range_searcher_ = std::make_shared<RangeSearcher>();
}

Frame::Frame(unsigned long timestamp, unsigned long id, const cv::Mat& image)
  : timestamp_(timestamp)
  , id_(id) 
  , image_(image){
  SetPose(cv::Mat::eye(4,4,CV_64F));
  range_searcher_ = std::make_shared<RangeSearcher>();
}

void Frame::PreProcess(std::shared_ptr<ORB_SLAM2::ORBextractor> extractor,std::shared_ptr<PinholeCamera> camera_model) {
  // Extract ORB
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

  // Build gridding range searcher
  range_searcher_->BuildGrids(undistorted_kps_, camera_model->image_bounds(), 2);

  // Allocate landmarks
  landmarks_ = std::vector<std::shared_ptr<Landmark>>(keypoints_.size(), static_cast<std::shared_ptr<Landmark>>(NULL));
}

cv::Mat Frame::image() const {
  return image_;
}

std::vector<cv::KeyPoint> Frame::keypoints() const {
  return keypoints_;
}

cv::Mat Frame::descriptors() const {
  return descriptors_;
}

std::vector<std::shared_ptr<Landmark>> Frame::landmarks() const {
  return landmarks_;
}

std::shared_ptr<RangeSearcher> Frame::range_searcher() const {
  return range_searcher_;
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
  T_cl_ = T_cl;
}

void Frame::AddLandmark(std::shared_ptr<Landmark> landmark, size_t idx) {
  landmarks_[idx] = landmark;
}

// TODO: consider if we need deep clone?
cv::Mat Frame::T_cw() const {
  return T_cw_;
}

cv::Mat Frame::T_wc() const {
  return T_wc_;
}

cv::Mat Frame::T_cl() const {
  return T_cl_;
}

cv::Mat Frame::Project(const cv::Mat pt3d_w) {
  CHECK(pt3d_w.cols == 1 && pt3d_w.rows == 3) << "Invalid 3d point.";
  std::cout<<"T_cw:\n"<<T_cw_<<std::endl;
  std::cout<<"R_cw:\n"<<R_cw_<<std::endl;
  CHECK(T_cw_.data) << "The pose of frame is NOT set.";
  cv::Mat p_c = R_cw_*pt3d_w + t_cw_;
  return p_c;
}

} // namespace lslam
