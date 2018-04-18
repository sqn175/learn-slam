/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#include "params_man.h"

#include <opencv2/core/core.hpp>
#include <glog/logging.h>

#include "my_assert.h"

namespace lslam {

ParamsMan::ParamsMan() {

}

ParamsMan::~ParamsMan() {
  
}
/**
 * @brief Read and parse the configuration xml file.
 * 
 * @param file_name configuration xml file name.
 */
void ParamsMan::Read(const std::string &file_name) {
  // Opencv file storages
  cv::FileStorage config_file(file_name, cv::FileStorage::READ);
  
  CHECK(config_file.isOpened()) << "Configuration file open failed.";
  
  LOG(INFO) << "Opened configuration file: " << file_name;
  
  cv::FileNode camera_params = config_file["camera"];
  
  CHECK(camera_params.size() == 6) << "Incomplete calibration in config file.";
  
  // TODO:: assert node
  cv::FileNode image_dimension_node = camera_params["image_dimension"];
  pinholecamera_params.img_width = (int)image_dimension_node[0];
  pinholecamera_params.img_height = (int)image_dimension_node[1];

  cv::FileNode distortion_coefficients_node = camera_params["distortion_coefficients"];
  pinholecamera_params.k1 = (double)distortion_coefficients_node[0];
  pinholecamera_params.k2 = (double)distortion_coefficients_node[1];
  pinholecamera_params.p1 = (double)distortion_coefficients_node[2];
  pinholecamera_params.p2 = (double)distortion_coefficients_node[3];

  pinholecamera_params.distortion_type = (std::string)(camera_params["distortion_type"]);
  
  cv::FileNode focal_length_node = camera_params["focal_length"];
  pinholecamera_params.fx = (double)focal_length_node[0];
  pinholecamera_params.fy = (double)focal_length_node[1];

  cv::FileNode principal_point_node = camera_params["principal_point"];
  pinholecamera_params.cx = (double) principal_point_node[0];
  pinholecamera_params.cy = (double) principal_point_node[1];

  pinholecamera_params.frame_rate = (int)(camera_params["camera_rate"]);
  
  cv::FileNode feature_params_filenode = config_file["features"];
  feature_params.n_feat = (int)feature_params_filenode["feature_numbers"];
  feature_params.scale_factor = (double)feature_params_filenode["scale_factor"];
  feature_params.levels = (int)feature_params_filenode["level"];
  feature_params.init_th_Fast = (int)feature_params_filenode["ini"];
  feature_params.min_th_Fast = (int)feature_params_filenode["min"];

  // Close the file
  config_file.release();
}

ParamsMan& ParamsMan::Instance() {
  static ParamsMan t;
  return t;
}

} // namespace lslam
