/*
 * Author: ShiQin
 * Date: 2017-08-17
 */

#ifndef FRONTEND_CV_H_
#define FRONTEND_CV_H_

#include <vector>
#include <opencv2/core.hpp>

namespace lslam {

// Score the Homography matrix H by evaluate the reprojection errors
double ScoreHomography(const std::vector<cv::Point2f> src_points, 
                       const std::vector<cv::Point2f> dst_points,
                       const cv::Mat H);

} // namespace lslam

#endif // FRONTEND_CV_H_
