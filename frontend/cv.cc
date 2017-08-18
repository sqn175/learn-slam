
#include "cv.h"
#include "../common/my_assert.h"

namespace lslam {

double ScoreHomography(const std::vector<cv::Point2f> src_points, 
                       const std::vector<cv::Point2f> dst_points,
                       const cv::Mat H) {
  
  ASLAM_ASSERT_TRUE( src_points.size() == dst_points.size(), "Size inconsistency");
  ASLAM_ASSERT_TRUE( H.cols == 3 && H.rows == 3, "Invalid homography size");
  
  double score = 0.0;
  
  cv::Mat H_inv = H.inv();
  
  auto it_src = src_points.begin();
  auto it_dst = dst_points.begin();
  for (; it_src != src_points.end(); ++it_src, ++it_dst) {
    const double u1 = (*it_src).x;
    const double v1 = (*it_src).y;
    const double u2 = (*it_dst).x;
    const double v2 = (*it_dst).y;
    
    // Reproject src_points to dst_points
    // p1in2 = H * p1
    double w1in2 = H.at<double>(2,0)*u1 + H.at<double>(2,1)*v1 + H.at<double>(2,2);
    ASLAM_ASSERT_TRUE( w1in2 != 0, "w1in2 should not be 0.")
    
    double u1in2 = (H.at<double>(0,0)*u1 + H.at<double>(0,1)*v1 + H.at<double>(0,2)) / w1in2;
    double v1in2 = (H.at<double>(1,0)*u1 + H.at<double>(1,1)*v1 + H.at<double>(1,2)) / w1in2;
    // Reprojection error
    double square_dist1 = (u2 - u1in2)*(u2 - u1in2) + (v2 - v1in2)*(v2 - v1in2);
    
    score += square_dist1;
    
    // Reproject dst_points to src_points
    // p2in1 = H_inv * p2;
    double w2in1 = H_inv.at<double>(2,0)*u2 + H_inv.at<double>(2,1)*v2 + H_inv.at<double>(2,2);
    ASLAM_ASSERT_TRUE( w2in1 != 0, "w2in1 should not be 0.");
    
    double u2in1 = (H_inv.at<double>(0,0)*u2 + H_inv.at<double>(0,1)*v2 + H_inv.at<double>(0,2)) / w2in1;
    double v2in1 = (H_inv.at<double>(1,0)*u2 + H_inv.at<double>(1,1)*v2 + H_inv.at<double>(1,2)) / w2in1;
    // Reprojection error
    double square_dist2 = (u1 - u2in1)*(u1 - u2in1) + (v1 - v2in1)*(v1 - v2in1);
    
    score += square_dist2;
  }
  
  return score;
  
}

} // namespace lslam
