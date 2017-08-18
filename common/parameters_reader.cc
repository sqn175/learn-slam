/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#include "parameters_reader.h"

#include <opencv2/core/core.hpp>
#include <glog/logging.h>

namespace lslam {

// Read and parse a yaml/xml config file
void ParamtersReader::Read(std::string file_name){
  // Opencv file storages
  cv::FileStorage config_file(file_name, cv::FileStorage::READ);
  // TODO: assert file open
  LOG(INFO) << "Opened configuration file: " << file_name;
  
  cv::FileNode camera_params = config_file["camera"];
  
}

} // namespace lslam
