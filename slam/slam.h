/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:19:03 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-10-25 20:32:13
 */

#ifndef SLAM_SLAM_H_
#define SLAM_SLAM_H_

#include <thread>
#include <string>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

#include "glog/logging.h"

#include "../3rdparty/ORB_SLAM2_modified/ORBextractor.h"
#include "params_man.h"
#include "pinhole_camera.h"
#include "threadsafe_queue.h"
#include "frontend.h"
#include "mapper.h"
#include "visualizer.h"

namespace lslam {

class Map;
class Frame;
class KeyFrame;
class Mapper;

struct RawData {
  cv::Mat image;
  double timestamp;
};

// SLAM as a system interface 
class Slam {
public:

  // Construct from a configuration file
  explicit Slam(const std::string config_file);
  // Slam is neither copyable nor movable
  Slam(const Slam&) = delete;
  Slam& operator=(const Slam&) = delete;

  ~Slam();
  
  // Initialize Slam system, including:
  // - Launch theads: tracking thread
  // - Initialize visualization routine
  void Init(const std::string config_file);

  // Feed a monocular image frame to system and process it.
  // Input: monocular RGB(CV_8UC3) or grayscale (CV_8U) image
  // Output: Camera world coordinate pose
  bool AddMonoImage(const cv::Mat &image, const double &timestamp);

  void ShutDown();

  void SaveTrajectory(const std::string& filename);
private:
  void FrameConsumerLoop();
  void VisualizationLoop();
  void MapperLoop();
  void OptimizationLoop();

private:
  //// Thread safe members
  // Map shared by all threads
  std::shared_ptr<Map> map_;
  // Queues shared by multi-threads
  // - Camera Measurement input queues
  ThreadSafeQueue<RawData> input_images_;
  // - Tracked result queues, ready to be displayed
  std::shared_ptr<ThreadSafeQueue<VisualizedData>> visualization_data_;
  // - Keyframes to be optimized
  ThreadSafeQueue<std::shared_ptr<KeyFrame>> keyframes_;

  // Operations
  std::shared_ptr<PinholeCamera> camera_model_;//Camera model
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;// ORB extractor to detect and describe features
  std::shared_ptr<ORBVocabulary> orb_voc_;// ORB Vocabulary
  std::shared_ptr<GuidedMatcher> guided_matcher_;// Guided Matcher

  // Threads
  std::thread frame_consumer_thread_; // Thread running FrameConsumerLoop
  std::thread visualization_thread_; // Thread running VisualizationLoop()
  std::thread mapper_thread_; // Thread running OptimizationLoop()

  unsigned long last_added_camerameas_time_; // Timestamp of the newest camera measurement added to camera_meas_received_
  unsigned long last_added_camerameas_id_; // id


  Frontend frontend_; // The frontend
  Mapper mapper_;

  // Lists used to recover the full camera trajectory at the end of execution
  // Basically we store the reference keyframe for each frame and its relative transformation
  std::vector<cv::Mat> relative_frame_poses_; 
  std::vector<std::shared_ptr<KeyFrame>> ref_keyframes_;
  std::vector<double> frame_timestamps_;
  
};
    
} // namespace lslam
 
 #endif
