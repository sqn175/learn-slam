/*
 * @Author: Shi Qin 
 * @Date: 2017-09-22 16:15:16 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 17:07:52
 */
#ifndef COMMON_VISUALIZER_H_
#define COMMON_VISUALIZER_H_

#include <memory>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>

#include "threadsafe_queue.h"

namespace lslam {
class Map;
class Frame;

struct VisualizedData {
  cv::Mat image;
  std::shared_ptr<Frame> frame;
};

class Visualizer {
public:
  Visualizer(std::shared_ptr<ThreadSafeQueue<VisualizedData>> queue, std::shared_ptr<Map> map);

  void Run();

  void DrawKeyFrames();
  void DrawLandmarks();

  pangolin::OpenGlMatrix GetCurrentOpenGlCameraMatrix(std::shared_ptr<Frame> cur_frame);

  void DrawCurrentFrame(pangolin::OpenGlMatrix);
private:
  std::shared_ptr<Map> map_; // Thread_safe map to be drawn

  // A reference of camera_meas_visualized_
  std::shared_ptr<ThreadSafeQueue<VisualizedData>> frame_queue_;
  //ThreadSafeQueue<VisualizedData>& frame_queue_;
  
  // render settings
  bool settings_followcamera;
  bool settings_showlandmarks;
  bool settings_showkeyframes;

  // Fps counter
  int n_frames_;
  std::chrono::steady_clock::time_point fps_t_start_;
  std::chrono::steady_clock::time_point fps_t_end_;
  std::string fps_str_;
};
} // namespace LSLAM

#endif  //COMMON_VISUALIZER_H_
