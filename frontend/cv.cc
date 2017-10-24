#include "cv.h"

#include "frame.h"

namespace lslam {

// Using ORB_SLAM initilizer class to calculate the camera ego motion given 2d2d matches
bool ComputeEgomotion(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2,
                      std::vector<cv::DMatch> matches, 
                      cv::Mat& R, cv::Mat&t, 
                      std::vector<cv::Point3f>& points_3d, std::vector<bool>& is_triangulated) {

  std::shared_ptr<ORB_SLAM2::Initializer> initializer;
  cv::Mat K = frame1->camera_model()->K().clone();
  // ORB_SLAM2 initializer only accept a CV_32F Mat, so we wrap K
  K.convertTo(K, CV_32F);
  initializer = std::make_shared<ORB_SLAM2::Initializer>(frame1->undistorted_kps(), K, 1.0, 200);
  std::vector<int> matches12(frame1->undistorted_kps().size(), -1);
  for (auto& match : matches) {
    matches12[match.queryIdx] = match.trainIdx;
  }
  // Test 
  if (frame2->id() == 25) {
    int test = 1;
    int test2 = test;
  }
  bool success =  initializer->Initialize(frame2->undistorted_kps(), matches12, R, t, points_3d, is_triangulated);
  return success;
}

} // namespace lslam
