/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include "frontend.h"

namespace lslam {

Frontend::Frontend() : state(FrontEndState::kNotReady) {
}

void Frontend::AddCameraMeasurement(CameraMeasurement& camera_measurement_current) {
  
  // if initialized 
  if (state == FrontEndState::kInitialized) {
    // TRACK
    
  } else if (state == FrontEndState::kNotInitialized) {
    
    // Find correspondences
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    // KNN matcher, k = 2, find the best match and the second best match
    matcher.knnMatch(camera_measurement_prev.descriptors(), camera_measurement_current.descriptors(), matches, 2);
    
    std::vector<cv::DMatch> good_matches;
    
    // The ratio threshold, we may adjust this
    // Kick out the correspondences,
    // which distance ratio of best match and second best match is below the threshold
    const double ratio = 0.8;
    
    for ( auto it = matches.begin(); it != matches.end(); ++it){
      if ((*it)[0].distance < 50 && (*it)[0].distance < ratio * (*it)[1].distance)
        good_matches.push_back((*it)[0]);
    }
    
    size_t num_correspondences = good_matches.size();
    
    if (num_correspondences < 10) {
      // Too few correspondences
      camera_measurement_prev = camera_measurement_current;
      return;
    }
    
    // convert keypoint to vecctor<Point2f>
    std::vector<cv::Point2f> src_points, dst_points;
    for ( auto it = good_matches.begin(); it != good_matches.end(); ++it){
      src_points.push_back( camera_measurement_prev.keypoints()[(*it).queryIdx].pt );
      dst_points.push_back( camera_measurement_current.keypoints()[(*it).trainIdx].pt );
    }
    
    // Find fundamental 
    cv::Mat K = camera_measurement_current.camera_model()->K();
    cv::Mat fundamental_matrix = cv::findFundamentalMat(src_points, dst_points, cv::FM_RANSAC, 3.841, 0.99, mask_f);
    cv::Mat essential_matrix = K.t() * fundamental_matrix * K;

    // Recover relative camera pose from essential matrix
    cv::Mat R, t;
    cv::recoverPose(essential_matrix, src_points, dst_points, K, R, t, cv::noArray());

    // Get quality
    int inlier_cnt = cv::countNonZero(mask_f);
    double inlier_ratio = double(inlier_cnt) / double(num_correspondences);
    
    if (inlier_ratio > 0.8) {
      // result is good
      state = FrontEndState::kInitialized;
      // Initialize the camera relative pose
      cv::Mat T_cw = cv::Mat::eye(4,4,CV_32F);
      R.copyTo(T_cw.rowRange(0,3).colRange(0,3));
      t.copyTo(T_cw.rowRange(0,3).col(3));
      camera_measurement_current.SetPose(T_cw);
    } 
    
    camera_measurement_prev = camera_measurement_current;
    
  } else if (state == FrontEndState::kNotReady) {
      camera_measurement_prev = camera_measurement_current;
      state = FrontEndState::kNotInitialized;
      return;
  }
  
}

} // namespace lslam
