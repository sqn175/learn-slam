/*
 * @Author: Shi Qin 
 * @Date: 2017-09-22 10:33:31 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 17:42:41
 */

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "slam.h"

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);

  // configuration file
  std::string config_file_name = "/home/sqn/Documents/learn-slam/config/config_fpga_p2_euroc.yaml";

  // Create slam system and initialize from configuration file
  // It initialize frontend and backend, start all system threads, and get ready to process incoming images
  lslam::Slam slam(config_file_name);

  // data path
  std::string images_file_name = "/home/sqn/SLAM_Data/EuroC_dataset_MH_3_medium/cam0/data/";
  std::string timestamp_file_name = "/home/sqn/SLAM_Data/EuroC_dataset_MH_3_medium/cam0/data.csv";
  
  std::vector<double> timestamps;
  std::vector<std::string> image_str;

  // Load image timestamps and image filenames 
  std::ifstream file_timestamps;
  file_timestamps.open(timestamp_file_name);
  if (!file_timestamps.good()) {
    LOG(ERROR) << "No timestamp file found at" << timestamp_file_name;
    return -1;
  }
  std::string line;
  // set reading position to second line
  std::getline(file_timestamps, line);
  while (std::getline(file_timestamps, line)) {
    std::stringstream stream(line);
    std::string s;
    std::getline(stream, s, ',');
    // timestamp in second
    timestamps.push_back(std::stod(s) / 1e9);
    std::getline(stream, s, '\r');
    image_str.push_back(s);
  }

  // Display info
  std::cout<<"No. images: "<<image_str.size()<<std::endl;

  // Feed image to slam system
  for (int i = 0; i < image_str.size(); ++i) {
    cv::Mat image = cv::imread(images_file_name + image_str[i], CV_LOAD_IMAGE_UNCHANGED);
    // test
    if (timestamps[i] < 1403637152138319104/1e9) continue;
    slam.AddMonoImage(image, timestamps[i]);

  }
  
  return 0;
}