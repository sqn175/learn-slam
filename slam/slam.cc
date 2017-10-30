/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:16:10 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 20:34:06
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

  // Initialize camera model
  // TODO: Check pinholecamera_params != null
  // Smart pointer initialize
  camera_model_ = std::make_shared<PinholeCamera>(params_.pinholecamera_params());
  
  // Initialize ORB extractor
  // TODO: using params to initialize
  int features = 2000;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  orb_extractor_ = std::make_shared<ORB_SLAM2::ORBextractor>(features, scale, level, ini, min);
  
  // Loading ORB vocabulary
  std::cout<<"Loading ORB Vocabulary..."<<std::endl;
  orb_voc_ = std::make_shared<ORBVocabulary>();
  bool voc_loaded = orb_voc_->loadFromTextFile("/home/sqn/Documents/learn-slam/Vocabulary/ORBvoc.txt");
  CHECK(voc_loaded) << "ORB Vocabulary loaded failed.";

  // Initialize guided matcher
  guided_matcher_ = std::make_shared<GuidedMatcher>(camera_model_,orb_extractor_);

  // Allocate
  map_ = std::make_shared<Map>();

  // Initialize frontend via params_
  frontend_ = Frontend(map_, camera_model_, orb_extractor_, orb_voc_, guided_matcher_);

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
  RawData input_data = {image, timestamp};
  
  camera_meas_received_.PushBlockingIfFull(input_data, 1);
} 

void Slam::FrameConsumerLoop() {
  RawData input_data;
  VisualizedData vis;
  cv::Mat im;
  std::shared_ptr<Frame> frame;
  while (true) {
    // Get data from queue, and check for termination check
    if (camera_meas_received_.PopBlocking(&input_data) == false)
      return;

    // Feed to frontend
    frontend_.Process(input_data.image, input_data.timestamp);
    frontend_.PublishVisualization(im, frame);

    vis.frame = frame;
    vis.image = im;
    // We get the current camera pose, push to the visualization_queue
    camera_meas_visualized_.PushBlockingIfFull(vis, 1);
  }
}

void Slam::VisualizationLoop() {
  Visualizer visualizer(camera_meas_visualized_, map_);
  visualizer.Run();
}

void Slam::MapperLoop() {
  std::shared_ptr<KeyFrame> keyframe;
  while (true) {
    if (keyframes_.PopBlocking(&keyframe) == false)
      return;

    // Feed to mapper
    mapper_.Process(keyframe);
  }
}

} // namespace lslam
