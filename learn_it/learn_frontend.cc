/*
 * Author: ShiQin
 * Date: 2017-08-21
 * 
 * This file shows the SLAM frontend routine
 * 
 */

#include <string>
#include <memory>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ORBextractor.h>
#include "params_man.h"
#include "pinhole_camera.h"
#include "camera_measurement.h"
#include "my_assert.h"
#include "frontend.h"
#include "MapDrawer.h"

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);

  // configuration file
  std::string config_file_name = "/home/sqn/Documents/learn-slam/config/config_fpga_p2_euroc.yaml";
  lslam::ParamsMan parameters_reader(config_file_name);
  
  //
  auto pinhole_camera = std::make_shared<const lslam::PinholeCamera>(parameters_reader.pinholecamera_params());
  
  int features = 100;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  auto orb_extractor = std::make_shared<ORB_SLAM2::ORBextractor>(features, scale, level, ini, min);
  
  // image from EuroC dataset
  std::string file_name = "//home//sqn//SLAM_Data//EuroC_dataset_MH_3_medium//cam0//data";
  // Extract the ORB features of image1
  cv::Mat image_1 = cv::imread(file_name + "//1403637217688318976.png");
  ASLAM_ASSERT_TRUE( image_1.data, "Could not open or find image.");
  cv::Mat image_gray_1;
  cv::cvtColor(image_1, image_gray_1, cv::COLOR_BGR2GRAY);
  auto frame_prev = std::make_shared<lslam::CameraMeasurement>(1, image_gray_1, pinhole_camera, orb_extractor);
  
  cv::Mat image_2 = cv::imread(file_name + "//1403637217738319104.png");
  ASLAM_ASSERT_TRUE( image_2.data, "Could not open or find image.");
  cv::Mat image_gray_2;
  cv::cvtColor(image_2, image_gray_2, cv::COLOR_BGR2GRAY);
  
  auto frame_current = std::make_shared<lslam::CameraMeasurement> (2, image_gray_2, pinhole_camera, orb_extractor);
  
  lslam::Frontend frontend;
  frontend.AddCameraMeasurement(frame_prev);
  frontend.AddCameraMeasurement(frame_current);
  
  return 0 ;
  
}
