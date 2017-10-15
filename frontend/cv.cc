#include "cv.h"


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
  std::vector<int> matches12;
  matches12.reserve(matches.size());
  for (int i = 0; i < matches.size(); ++i) {
    if (matches[i].trainIdx >=0) {
      matches12.push_back(matches[i].trainIdx);
    } else {
      matches12.push_back(-1);
    }
  }
  return initializer->Initialize(frame2->undistorted_kps(), matches12, R, t, points_3d, is_triangulated);
}

} // namespace lslam
