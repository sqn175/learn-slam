/*
 * @Author: Shi Qin 
 * @Date: 2017-09-22 16:15:16 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 17:07:52
 */
#ifndef COMMON_VISUALIZER_H_
#define COMMON_VISUALIZER_H_

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>

#include "frame.h"
#include "threadsafe_queue.h"
#include "map.h"

namespace lslam {

class Visualizer {
public:
  Visualizer(ThreadSafeQueue<std::shared_ptr<Frame>>& queue, std::shared_ptr<Map> map);

  void Run();

  void DrawKeyFrames();
  void DrawLandmarks();

  pangolin::OpenGlMatrix GetCurrentOpenGlCameraMatrix(std::shared_ptr<Frame> cur_frame);

  void DrawCurrentFrame(pangolin::OpenGlMatrix);
  
  cv::Mat DrawFrame(std::shared_ptr<Frame>);
private:
  std::shared_ptr<Map> map_; // Thread_safe map to be drawn

  // A reference of camera_meas_visualized_
  ThreadSafeQueue<std::shared_ptr<Frame>>& frame_queue_;
  
  // render settings
  bool settings_followcamera;
  bool settings_showlandmarks;
  bool settings_showkeyframes;
};
} // namespace LSLAM

#endif  //COMMON_VISUALIZER_H_
