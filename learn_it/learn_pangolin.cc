/*
 * Author: ShiQin
 * Date: 2017-09-04
 * 
 * This file show the frontend initialization result using Pangolin
 * 
 */

#include <string>
#include <memory>
#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ORBextractor.h>
#include "parameters_reader.h"
#include "pinhole_camera.h"
#include "camera_measurement.h"
#include "my_assert.h"
#include "frontend.h"
#include "MapDrawer.h"
#include "Viewer.h"

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);

  // configuration file
  std::string config_file_name = "/home/sqn/Documents/learn-slam/config/config_fpga_p2_euroc.yaml";
  lslam::ParametersReader parameters_reader(config_file_name);
  
  //
  auto pinhole_camera = std::make_shared<const lslam::PinholeCamera>(parameters_reader.pinholecamera_params());
  
  int features = 1000;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  auto orb_extractor = std::make_shared<ORB_SLAM2::ORBextractor>(features, scale, level, ini, min);
  
  // image from EuroC dataset
  std::string file_name = "//home//sqn//SLAM_Data//EuroC_dataset_MH_3_medium//cam0//data";
  // Extract the ORB features of image1
  cv::Mat image_1 = cv::imread(file_name + "//1403637156988318976.png");
  ASLAM_ASSERT_TRUE( image_1.data, "Could not open or find image.");
  cv::Mat image_gray_1;
  cv::cvtColor(image_1, image_gray_1, cv::COLOR_BGR2GRAY);
  auto frame_prev = std::make_shared<lslam::CameraMeasurement>(1, image_gray_1, pinhole_camera, orb_extractor);
  
  cv::Mat image_2 = cv::imread(file_name + "//1403637157088318976.png");
  ASLAM_ASSERT_TRUE( image_2.data, "Could not open or find image.");
  cv::Mat image_gray_2;
  cv::cvtColor(image_2, image_gray_2, cv::COLOR_BGR2GRAY);
  
  auto frame_current = std::make_shared<lslam::CameraMeasurement> (2, image_gray_2, pinhole_camera, orb_extractor);
  
  lslam::Frontend frontend;
  frontend.AddCameraMeasurement(frame_prev);
  frontend.AddCameraMeasurement(frame_current);
  
  auto map_drawer = std::make_shared<ORB_SLAM2::MapDrawer>(frontend.map(),config_file_name);
  
  // test, write world_coordinates to text file for matlab ploting
  std::ofstream matlab_points_file;
  matlab_points_file.open("pointfile.txt");
  const std::vector<std::shared_ptr<lslam::Landmark>> vpMPs = frontend.map()->landmarkpoints();
  for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
  {
    cv::Mat x = vpMPs[i]->point_world();
    std::cout<<x<<std::endl;
    matlab_points_file << x.at<double>(0,0) << " " << x.at<double>(1,0) << " " << x.at<double>(2,0) << "\n";
  }

  matlab_points_file.close();
  
  std::shared_ptr<ORB_SLAM2::Viewer> viewer = std::make_shared<ORB_SLAM2::Viewer>(map_drawer,config_file_name);
  viewer->Run();
  
  return 0 ;
  
}
