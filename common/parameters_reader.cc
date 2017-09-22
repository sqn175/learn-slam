/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#include "parameters_reader.h"

#include <opencv2/core/core.hpp>
#include <glog/logging.h>

#include "my_assert.h"

namespace lslam {

ParametersReader::ParametersReader(const std::string &file_name) {
  Read(file_name);
}

// Read and parse a yaml/xml config file
void ParametersReader::Read(const std::string &file_name) {
  // Opencv file storages
  cv::FileStorage config_file(file_name, cv::FileStorage::READ);
  
  ASLAM_ASSERT_TRUE(config_file.isOpened(), "Configuration file open failed.")
  
  LOG(INFO) << "Opened configuration file: " << file_name;
  
  cv::FileNode camera_params = config_file["camera"];
  
  ASLAM_ASSERT_TRUE(camera_params.size() == 6, "Incomplete calibration in config file.");
  
  // TODO:: assert node
  cv::FileNode image_dimension_node = camera_params["image_dimension"];
  pinholecamera_params_.image_dimension << image_dimension_node[0], image_dimension_node[1];
  cv::FileNode distortion_coefficients_node = camera_params["distortion_coefficients"];
  pinholecamera_params_.distortion_coef.resize(distortion_coefficients_node.size());
  for (size_t i = 0; i < distortion_coefficients_node.size(); ++i) {
    pinholecamera_params_.distortion_coef[i] = distortion_coefficients_node[i];
  }
  pinholecamera_params_.distortion_type = (std::string)(camera_params["distortion_type"]);
  cv::FileNode focal_length_node = camera_params["focal_length"];
  pinholecamera_params_.focal_length << focal_length_node[0], focal_length_node[1];
  cv::FileNode principal_point_node = camera_params["principal_point"];
  pinholecamera_params_.principal_point << principal_point_node[0], principal_point_node[1];
  pinholecamera_params_.frame_rate = (int)(camera_params["camera_rate"]);
  
  // Close the file
  config_file.release();
}

PinholeCameraParameters ParametersReader::pinholecamera_params() const {
  return pinholecamera_params_;
}

} // namespace lslam
