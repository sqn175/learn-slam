/*
 * Author: ShiQin
 * Date: 2017-08-22
 */

#include "frontend.h"

namespace lslam {

Frontend::Frontend() : is_initialized_(false) {
}

void Frontend::AddCameraMeasurement(const CameraMeasurement camera_measurement_current) {
  
  // if we get the initialized camera pose
  if (is_initialized_) {
    // TRACK
  } else {// We get the initial camera pose by RANSAC 2d2d
    
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
    
    // convert keypoint to vecctor<Point2f>
    std::vector<cv::Point2f> src_points, dst_points;
    for ( auto it = good_matches.begin(); it != good_matches.end(); ++it){
      src_points.push_back( keypoints_1[(*it).queryIdx].pt );
      dst_points.push_back( keypoints_2[(*it).trainIdx].pt );
    }
    
    // Find fundamental 
    cv::Mat K = camera_measurement_current.camera_model()->K();
    cv::Mat fundamental_matrix = cv::findFundamentalMat(src_points, dst_points, cv::FM_RANSAC, 3.841, 0.99, cv::noArray());
    cv::Mat essential_matrix = K.t() * fundamental_matrix * K;
    
    // Recover relative camera pose from essential matrix
    cv::Mat R, t, mask;
    cv::recoverPose(essential_matrix, src_points, dst_points, K, R, t, mask);
    
    
    
  }
  
  // Update camera measurement
  camera_measurement_prev = camera_measurement;
  
}

} // namespace lslam
