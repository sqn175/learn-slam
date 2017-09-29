/*
 * @Author: Shi Qin 
 * @Date: 2017-09-26 14:47:33 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 16:55:34
 */

#ifndef COMMON_RANGE_SEARCHER_H_
#define COMMON_RANGE_SEARCHER_H_

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <glog/logging.h>

namespace lslam {

// 2D OpenCv keypoint orthogonal range searching, using gridding method
class RangeSearcher{
public:
  RangeSearcher() {}

  // 
  void BuildGrids(const std::vector<cv::KeyPoint>& points, const cv::Mat& bounds, int n_points_per_grid);
  
  // search_bounds: pt_top_left_u,  pt_top_left_v, pt_bottom_right_u, pt_bottom_right_v 
  std::vector<size_t> PointsInRange(const cv::Mat& search_bounds);
private:
  // Row-major grids maintaining the features
  std::vector<std::vector<std::vector<size_t>>> grids_;
  // Number of grids
  int grids_cols_, grids_rows_;
  // 1 / Grid square size
  double grid_size_inv_;
  // Sorted points bounds
  double min_x_;
  double min_y_;
  double max_x_;
  double max_y_;
};
} // namespace LSLAM

#endif  // COMMON_RANGE_SEARCHER_H_
