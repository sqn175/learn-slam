#include "cv.h"

#include "frame.h"
#include "keyframe.h"

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

cv::Mat Skew(const cv::Mat&v) {
  CHECK(v.rows == 3 && v.cols == 1);
  CHECK(v.type() == 6); // type() = 6 -> CV_64F, double
  return (cv::Mat_<double>(3,3) <<               0, -v.at<double>(2),  v.at<double>(1),
                                   v.at<double>(2),                0, -v.at<double>(0),
                                  -v.at<double>(1),  v.at<double>(0),               0);
}

cv::Mat RecoverFundamental(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2) {
  cv::Mat R1w = frame1->R_cw();
  cv::Mat t1w = frame1->t_cw();
  cv::Mat R2w = frame2->R_cw();
  cv::Mat t2w = frame2->t_cw();

  cv::Mat R12 = R1w*R2w.t();
  cv::Mat t12 = -R1w*R2w.t()*t2w + t1w;

  cv::Mat t12x = Skew(t12);

  const cv::Mat& K1 = frame1->camera_model()->K();
  const cv::Mat& K2 = frame2->camera_model()->K();

  return K1.t().inv() * t12x*R12 * K2.inv();
}

bool CheckDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                           const cv::Mat& F12, const double th) {
  CHECK(F12.cols == 3 && F12.rows == 3);
  // Epipolar line
  // p1'*F*p2 = 0
  double x1 = kp1.pt.x;
  double y1 = kp1.pt.y;
  const double a = x1*F12.at<double>(0,0) + y1*F12.at<double>(1,0) + F12.at<double>(2,0);
  const double b = x1*F12.at<double>(0,1) + y1*F12.at<double>(1,1) + F12.at<double>(2,1);
  const double c = x1*F12.at<double>(0,2) + y1*F12.at<double>(1,2) + F12.at<double>(2,2);

  const double num = a*kp2.pt.x + b*kp2.pt.y + c;
  const double den = a*a + b*b;

  if (den == 0) 
    return false;
  
  const double dsqt = num*num / den;

  return dsqt < th;
}

} // namespace lslam
