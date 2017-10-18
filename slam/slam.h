/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:19:03 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 20:28:57
 */

#ifndef SLAM_SLAM_H_
#define SLAM_SLAM_H_

#include <thread>
#include <string>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "glog/logging.h"

#include "parameters_reader.h"
#include "pinhole_camera.h"
#include "threadsafe_queue.h"
#include "frontend.h"
// #include "visualizer.h"

namespace lslam {

class Map;
class Frame;
class KeyFrame;

// SLAM as a system interface 
class Slam {
public:

  // Construct from a configuration file
  explicit Slam(const std::string config_file);

  // Initialize Slam system, including:
  // - Launch theads: tracking thread
  // - Initialize visualization routine
  bool Init(const std::string config_file);

  // Feed a monocular image frame to system and process it.
  // Input: monocular RGB(CV_8UC3) or grayscale (CV_8U) image
  // Output: Camera world coordinate pose
  bool AddMonoImage(const cv::Mat &image, const double &timestamp);

private:
  void FrameConsumerLoop();
  void VisualizationLoop();
  void OptimizationLoop();

private:
  // Slam is neither copyable nor movable
  Slam(const Slam&) = delete;
  Slam& operator=(const Slam&) = delete;

  // Parameters reader
  ParametersReader params_;

  //// Thread safe members
  // Map shared by all threads
  std::shared_ptr<Map> map_;
  // Queues shared by multi-threads
  // - Camera Measurement input queues
  ThreadSafeQueue<std::shared_ptr<Frame>> camera_meas_received_;
  // - Tracked result queues, ready to be displayed
  ThreadSafeQueue<std::shared_ptr<Frame>> camera_meas_visualized_;
  // - Keyframes to be optimized
  ThreadSafeQueue<std::shared_ptr<KeyFrame>> keyframes_;

  // Threads
  std::thread frame_consumer_thread_; // Thread running FrameConsumerLoop
  std::thread visualization_thread_; // Thread running VisualizationLoop()
  std::thread optimization_thread_; // Thread running OptimizationLoop()

  unsigned long last_added_camerameas_time_; // Timestamp of the newest camera measurement added to camera_meas_received_
  unsigned long last_added_camerameas_id_; // id

  Frontend frontend_; // The frontend

};
    
} // namespace lslam
 
 #endif
