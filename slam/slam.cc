/*
 * @Author: Shi Qin 
 * @Date: 2017-09-19 10:16:10 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-12-05 20:34:24
 */

#include "slam.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>

#include <glog/logging.h>
#include "../3rdparty/ORB_SLAM2_modified/Converter.h"

#include "visualizer.h"
#include "map.h"
#include "frame.h"
#include "keyframe.h"
#include "time_logger.h"

namespace lslam {

Slam::Slam(const std::string config_file) {
  Init(config_file);
}

Slam::~Slam() {
  ShutDown();
}

void Slam::Init(const std::string config_file) {
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
  // TODO: use fast Bow https://github.com/rmsalinas/fbow
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

  // Initialize mapper
  mapper_ = Mapper(map_, guided_matcher_, orb_extractor_);

  visualization_data_ = std::make_shared<ThreadSafeQueue<VisualizedData>>();
  
  // Start threads
   mapper_thread_ = std::thread(&Slam::MapperLoop, this);
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
  
  input_images_.PushBlockingIfFull(input_data, 1);

  return true;
} 

/**
 * @brief This shutdown all threadsafe queues and request all threads to finish.
 * 
 */
void Slam::ShutDown() {
  input_images_.ShutDown();
  visualization_data_->ShutDown();
  keyframes_.ShutDown();
  
  // TODO: check these thread is joinable
  frame_consumer_thread_.join();
  visualization_thread_.join();
  mapper_thread_.join();
  
#ifdef USE_TIMER
  LOG(INFO) << Timing::Print();
#endif
}

void Slam::FrameConsumerLoop() {
  RawData input_data;
  VisualizedData vis;
  cv::Mat im;
  std::shared_ptr<Frame> frame;
  std::shared_ptr<KeyFrame> keyframe;
  while (true) {
    // Get data from queue, and check for termination check
    if (input_images_.GetCopyOfFrontBlocking(input_data) == false)
      return;

#ifdef USE_TIMER
    std::shared_ptr<TimeLogger> timer;
    if (frontend_.state() == Frontend::kNotInitialized) {
      timer = std::make_shared<TimeLogger>("1. frontend init process");
    } else {
      timer = std::make_shared<TimeLogger>("2. frontend track process");
    }
#endif

    // Feed to frontend
    frontend_.Process(input_data.image, input_data.timestamp);

    // pubish processed frame and image
    frame = frontend_.cur_frame();
    im = frontend_.image();

    // Visualization
    vis.frame = frame;
    vis.image = im;
    visualization_data_->PushBlockingIfFull(vis, 1);
    
    // 
    if (frontend_.state() == Frontend::kInitialized) {
      // Initialized, we create the first two keyframes
      keyframes_.PushBlockingIfFull(frontend_.init_keyframe(), 1);
      keyframes_.PushBlockingIfFull(frontend_.cur_keyframe(),1);
    } else if (frontend_.state() == Frontend::kTracking) {
      if (frontend_.FrameIsKeyFrame()) {
        if (keyframes_.Empty()) {
          frontend_.CreateKeyFrame();
          keyframes_.PushBlockingIfFull(frontend_.cur_keyframe(),1);
        } else {
          std::cout<<"Is keyframe but not empty."<<std::endl;
        }
      }
    }

#ifdef USE_TIMER
    timer->Stop();
#endif

    // Store frame pose information to retrieve the complete camera trajectory afterwards
    if (frontend_.state() == Frontend::FrontEndState::kInitialized || frontend_.state() == Frontend::FrontEndState::kTracking) {
      cv::Mat T_c_cr = frame->T_cw() * frontend_.cur_keyframe()->T_wc();
      relative_frame_poses_.push_back(T_c_cr);
      ref_keyframes_.push_back(frontend_.cur_keyframe());
      frame_timestamps_.push_back(frame->timestamp());
    }

    input_images_.PopNonBlocking();
  }
}

void Slam::VisualizationLoop() {
  Visualizer visualizer(visualization_data_, map_);
  visualizer.Run();
}

void Slam::MapperLoop() {
  std::shared_ptr<KeyFrame> keyframe;
  while (true) {
    if (keyframes_.GetCopyOfFrontBlocking(keyframe) == false)
      return;
#ifdef USE_TIMER
    TimeLogger timer("3. mapper process");
#endif
    // Feed to mapper
    mapper_.Process(keyframe);
#ifdef USE_TIMER
    timer.Stop();
#endif

    keyframes_.PopNonBlocking();
  }
}

void Slam::SaveTrajectory(const std::string& filename) {
  LOG(INFO) << "Saving trajectory to " << filename;
  std::ofstream f;
  f.open(filename.c_str());

  f << std::fixed;
  
  CHECK(relative_frame_poses_.size() == ref_keyframes_.size() && 
        ref_keyframes_.size() == frame_timestamps_.size());

  auto kfs = map_->keyframes();
  cv::Mat T_w_origin = map_->keyframes().front()->T_wc();
  // TODO: check this is the origin keyframe
  CHECK(map_->keyframes().front()->id() == 0);

  for (size_t i = 0; i < relative_frame_poses_.size(); ++i) {
    std::shared_ptr<KeyFrame> kf = ref_keyframes_[i];
    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    cv::Mat T_c_rp = cv::Mat::eye(4,4,CV_64F);
    while (kf->is_bad()) {
      // Transformation matrix from reference keyframe's parent keyframe to reference keyframe
      cv::Mat T_c_rp_tmp = kf->T_cw() * kf->parent_keyframe()->T_wc();
      T_c_rp = T_c_rp * T_c_rp_tmp;
      kf = kf->parent_keyframe();
    }
    cv::Mat T_cw_ro = T_c_rp * kf->T_cw() * T_w_origin;
    cv::Mat T_cw_c = relative_frame_poses_[i] * T_cw_ro;
    cv::Mat R = T_cw_c.rowRange(0,3).colRange(0,3).t();
    // Camera centre in world coordinate, this is the position.
    cv::Mat t = -R*T_cw_c.rowRange(0,3).col(3);

    std::vector<double> q = ORB_SLAM2::Converter::toQuaternion(R);
    f << std::setprecision(6) << frame_timestamps_[i] << "," << std::setprecision(9)
      << t.at<double>(0) << "," << t.at<double>(1) << "," << t.at<double>(2) << ","
      << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "\n";
  }
  f.close();
  LOG(INFO) << "Trajectory saved.";
}

} // namespace lslam
