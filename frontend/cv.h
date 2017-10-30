/*
 * Author: ShiQin
 * Date: 2017-08-17
 */

#ifndef FRONTEND_CV_H_
#define FRONTEND_CV_H_

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "../3rdparty/ORB_SLAM2_modified/Initializer.h"

namespace lslam {
class Frame;
class KeyFrame;

// 
bool ComputeEgomotion(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2,
                      std::vector<cv::DMatch> matches, 
                      cv::Mat& R, cv::Mat&t, 
                      std::vector<cv::Point3f>& points_3d, std::vector<bool>& is_triangulated);

//Returns the 3x3 skew symmetric matrix of a vector.
//v	Input 3x1 vector.
cv::Mat Skew(const cv::Mat&v);

// Recover fundamental function according to two frames' pose
// F12 = K1^(-T)*t12^*R12*K2^(-1)
// Return F12, pose of kf2 set as Identity
cv::Mat RecoverFundamental(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2);

bool CheckDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                           const cv::Mat& F12, const double th);

} // namespace lslam

#endif // FRONTEND_CV_H_
