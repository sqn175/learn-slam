/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:16:10 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-20 16:35:05
 */

#include "slam.h"

#include "parameters_reader.h"

namespace lslam {

Slam::Slam(const std::string config_file) {

}

bool Slam::Init(const std::string config_file) {
  // Read params from config_file
  params_.Read(config_file);

  // Initialize frontend via params_
  frontend_.init(params_);

  last_added_camerameas_time_ = 0;
  last_added_camerameas_id_ = 0;

  
}
bool Slam::AddMonoImage(const cv::Mat &image, const unsigned long &timestamp) {
  if (last_added_camerameas_time_ > timestamp) {
    LOG(ERROR) << "Received image timestamp from past.";
    return false;
  }
  last_added_camerameas_time_ = timestamp;
  last_added_camerameas_id_ += 1;
  auto camera_meas = std::make_shared<CameraMeasurement>(timestamp, last_added_camerameas_id_, image);
  
  camera_meas_received_.PushBlockingIfFull(camera_meas, 1);
} 

void Slam::FrameConsumerLoop() {
  std::shared_ptr<CameraMeasurement> camera_meas;
  while (true) {
    // Get data from queue, and check for termination check
    if (camera_meas_received_.PopBlocking(&camera_meas) == false)
      return;
    
    // Feed to frontend
    frontend_.Process(camera_meas);

    // We get the current camera pose, push to the visualization_queue
    camera_meas_visualized_.PushBlockingIfFull(camera_meas, 1);
  }
}

void Slam::VisualizationLoop() {
  
}

} // namespace lslam
