/*
 * Author: ShiQin
 * Date: 2017-08-14
 * 
 * This file shows how to extract the ORB features and match these features.
 * extractor are implemented in ORB_SLAM2_modified
 * 
 */

#include <iostream>
#include <memory>

//#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include "ORBextractor.h"
#include "cv.h"
#include "../common/my_assert.h"

int main(){
  
  // image from EuroC dataset
  std::string file_name = "//home//sqn//SLAM_Data//EuroC_dataset_MH_3_medium//cam0//data";
  // Extract the ORB features of image1
  cv::Mat image_1 = cv::imread(file_name + "//1403637217688318976.png");
  ASLAM_ASSERT_TRUE( image_1.data, "Could not open or find image.");
  cv::Mat image_gray_1;
  cv::cvtColor(image_1, image_gray_1, cv::COLOR_BGR2GRAY);
  
  cv::Mat image_2 = cv::imread(file_name + "//1403637217738319104.png");
  ASLAM_ASSERT_TRUE( image_2.data, "Could not open or find image.");
  cv::Mat image_gray_2;
  cv::cvtColor(image_2, image_gray_2, cv::COLOR_BGR2GRAY);
  
  // Initialize ORB extractor from ORB-SLAM2 
  int features = 100;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  ORB_SLAM2::ORBextractor extractor(features, scale, level, ini, min);

  // Extract ORB
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat descriptor_1, descriptor_2;
  (extractor)( image_gray_1, cv::Mat(), keypoints_1, descriptor_1);
  (extractor)( image_gray_2, cv::Mat(), keypoints_2, descriptor_2);
  
  // Draw ORB keypoints
  cv::Mat out_img_1, out_img_2;
  cv::drawKeypoints(image_gray_1, keypoints_1, out_img_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(image_gray_2, keypoints_2, out_img_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::Size sz1 = out_img_1.size();
  cv::Size sz2 = out_img_2.size();
  cv::Mat out_img_paral(sz1.height, sz1.width + sz2.width, CV_8UC3);
  out_img_1.copyTo(out_img_paral(cv::Rect(0,0,sz1.width,sz1.height)));
  out_img_2.copyTo(out_img_paral(cv::Rect(sz1.width,0,sz2.width,sz2.height)));
  imshow("ORB-SLAM2 features", out_img_paral);
  cv::waitKey(0);
  
  // Brute-Force matching 
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> matches;
  //matcher.match(descriptor_1, descriptor_2, matchers);
  matcher.knnMatch(descriptor_1,descriptor_2, matches, 2);
  cv::Mat img_match;
  cv::drawMatches(image_gray_1, keypoints_1, image_gray_2, keypoints_2, matches, img_match);
  imshow("raw match", img_match);
  cv::waitKey(0);
  
  std::vector<cv::DMatch> good_matches;
  const double ratio = 0.8;
  for ( auto it = matches.begin(); it != matches.end(); ++it){
    if ((*it)[0].distance < 50 && (*it)[0].distance < ratio * (*it)[1].distance)
      good_matches.push_back((*it)[0]);
  }
  cv::Mat img_good_match;
  cv::drawMatches(image_gray_1, keypoints_1, image_gray_2, keypoints_2, good_matches, img_good_match);
  imshow("good match", img_good_match);
  cv::waitKey(0);
  
  // convert keypoint to vecctor<Point2f>
  std::vector<cv::Point2f> src_points, dst_points;
  for ( auto it = good_matches.begin(); it != good_matches.end(); ++it){
    src_points.push_back( keypoints_1[(*it).queryIdx].pt );
    dst_points.push_back( keypoints_2[(*it).trainIdx].pt );
  }
  
  // find Homography
  cv::Mat mask_h;
  cv::Mat homography_matrix = cv::findHomography(src_points, dst_points, cv::RANSAC, 5, mask_h, 500, 0.99);
  std::cout<< mask_h << std::endl;
  
  // check homography 
  double score = lslam::ScoreHomography(src_points, dst_points, homography_matrix);
  
  // intrinsic camera calibration matrix
  cv::Mat K = (cv::Mat_<double>(3,3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
  
  // find Fundamental
  cv::Mat mast_f;
  cv::Mat fundamental_matrix = cv::findFundamentalMat(src_points, dst_points, cv::FM_RANSAC, 3, 0.99, mask_f);
  cv::Mat essential_matrix = K.t() * fundamental_matrix * K;
  
  

  std::vector<cv::Mat> rotations, translations, normals;
  int test = cv::decomposeHomographyMat(homography_matrix, K, rotations, translations, normals);
  
  std::cout<<rotations[0]<<std::endl;
  std::cout<<normals[0]<<std::endl;

  
  return 0;
}
