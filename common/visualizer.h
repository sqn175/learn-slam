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

#include "frame.h"

namespace lslam {

class Visualizer {
public:
  Visualizer();

  void DrawFrameAndMap(std::string window_name, std::shared_ptr<Frame> camera_meas);
  
  cv::Mat DrawFrame(std::shared_ptr<Frame>);
};
} // namespace LSLAM

#endif  //COMMON_VISUALIZER_H_
