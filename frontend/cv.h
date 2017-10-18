/*
 * Author: ShiQin
 * Date: 2017-08-17
 */

#ifndef FRONTEND_CV_H_
#define FRONTEND_CV_H_

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include "../3rdparty/ORB_SLAM2_modified/Initializer.h"

namespace lslam {
class Frame;

// 
bool ComputeEgomotion(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2,
                      std::vector<cv::DMatch> matches, 
                      cv::Mat& R, cv::Mat&t, 
                      std::vector<cv::Point3f>& points_3d, std::vector<bool>& is_triangulated);
  

} // namespace lslam

#endif // FRONTEND_CV_H_
