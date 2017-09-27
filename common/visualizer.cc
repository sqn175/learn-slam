/*
 * @Author: Shi Qin 
 * @Date: 2017-09-22 16:18:35 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-22 17:48:04
 */

#include <vector>

#include "visualizer.h"

namespace lslam {

Visualizer::Visualizer() { }

void Visualizer::DrawFrameAndMap(std::string window_name, std::shared_ptr<Frame> camera_meas) {
  // Draw frame
  cv::Mat im = DrawFrame(camera_meas);
  cv::imshow(window_name, im);
  cv::waitKey(1);
}

cv::Mat Visualizer::DrawFrame(std::shared_ptr<Frame> camera_meas) {
  cv::Mat im = camera_meas->image();
  // Convert grayscale image to BGR image
  cv::cvtColor(im, im, CV_GRAY2BGR);

  std::vector<cv::KeyPoint> kps = camera_meas->keypoints();

  const double r = 5;
  const int size = kps.size();
  for (int i = 0; i < size; ++i) {
    cv::Point2d pt1, pt2;
    pt1.x = kps[i].pt.x - r;
    pt1.y = kps[i].pt.y - r;
    pt2.x = kps[i].pt.x + r;
    pt2.y = kps[i].pt.y + r;

    cv::rectangle(im, pt1, pt2, cv::Scalar(0,255,0));
  }

  return im;
}

} // namespace LSLAM
  