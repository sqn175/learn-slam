/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:16:10 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-25 09:38:10
 */

#include "slam.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "visualizer.h"
#include "map.h"
#include "frame.h"
#include "keyframe.h"

namespace lslam {

Slam::Slam(const std::string config_file) {
  Init(config_file);
}

bool Slam::Init(const std::string config_file) {
  // Read params from config_file
  params_.Read(config_file);

  // Allocate
  map_ = std::make_shared<Map>();

  // Initialize frontend via params_
  frontend_.init(params_);
  frontend_.set_map(map_);

  last_added_camerameas_time_ = 0;
  last_added_camerameas_id_ = 0;

  // Start threads
  frame_consumer_thread_ = std::thread(&Slam::FrameConsumerLoop, this);
  visualization_thread_ = std::thread(&Slam::VisualizationLoop, this);

}
bool Slam::AddMonoImage(const cv::Mat &image, const double &timestamp) {
  // Check image is valid
  CHECK(image.data) << "Invalid image input!";
  
  if (last_added_camerameas_time_ > timestamp) {
    LOG(ERROR) << "Received image timestamp from past.";
    return false;
  }
  last_added_camerameas_time_ = timestamp;
  last_added_camerameas_id_ += 1;
  
  // Wrap image to Frame
  auto camera_meas = std::make_shared<Frame>(timestamp, last_added_camerameas_id_, image);
  camera_meas_received_.PushBlockingIfFull(camera_meas, 1);
} 

void Slam::FrameConsumerLoop() {
  std::shared_ptr<Frame> camera_meas;
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
  Visualizer visualizer(camera_meas_visualized_, map_);
  visualizer.Run();
}

} // namespace lslam
