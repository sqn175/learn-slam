/*
 * @Author: Shi Qin 
 * @Date: 2017-09-26 15:24:05 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 17:13:00
 */

#include "range_searcher.h"

namespace lslam {

void RangeSearcher::BuildGrids(const std::vector<cv::KeyPoint>& points, const cv::Mat& bounds, int n_points_per_grid) {
  int n_points = points.size();
  int n_grids = std::ceil(n_points/n_points_per_grid);
  min_u_ = bounds.at<double>(0);
  min_v_ = bounds.at<double>(1);
  max_u_ = bounds.at<double>(2);
  max_v_ = bounds.at<double>(3);
  // height / width ratio
  double ratio = (max_u_ - min_u_) / (max_v_ - min_v_);
  grids_cols_ = std::ceil(std::sqrt((double)n_grids / ratio));
  grids_rows_ = std::ceil((double)n_grids / (double)grids_cols_);
  n_grids = grids_cols_*grids_rows_;
  // TODO: debug grid_size_inv_ = (double)grids_rows_ / (max_u - min_u)
  grid_size_inv_ = (double)grids_cols_ / (max_v_ - min_v_);

  // Assign points to grids
  for (int i = 0; i < n_points; ++i) {
    const cv::KeyPoint &kp = points[i];
    int grid_col = std::round( (kp.pt.x-min_v_)*grid_size_inv_ );
    int grid_row = std::round( (kp.pt.y-min_u_)*grid_size_inv_ );
    if (grid_col < 0 || grid_col >= grids_cols_ || grid_row < 0 || grid_row >= grids_rows_) {
      LOG(INFO) << "Assign feature to girds out of range.";
      continue;
    }
    grids_[grid_row][grid_col].push_back(i);
  }
}

std::vector<size_t> RangeSearcher::PointsInRange(const cv::Mat& search_bounds) {
  std::vector<size_t> indices;
  double min_u_search = search_bounds.at<double>(0);
  double min_v_search = search_bounds.at<double>(1);
  double max_u_search = search_bounds.at<double>(2);
  double max_v_search = search_bounds.at<double>(3);
  int min_grid_row = std::max(0, (int)std::floor((min_u_search - min_u_)*grid_size_inv_));
  if (min_grid_row >= grids_rows_) return indices;
  int min_grid_col = std::max(0, (int)std::floor((min_v_search - min_v_)*grid_size_inv_));
  if (min_grid_col >= grids_cols_) return indices;
  int max_grid_row = std::min(grids_rows_-1, (int)std::ceil((max_u_search - min_u_)*grid_size_inv_));
  if (max_grid_row < 0) return indices;
  int max_grid_col = std::min(grids_cols_-1, (int)std::ceil((max_v_search - min_v_)*grid_size_inv_));
  if (max_grid_col < 0) return indices;

  for (int i_row = min_grid_row; i_row <= max_grid_row; ++i_row) {
    for (int i_col = min_grid_col; i_col <= max_grid_col; ++i_col) {
      const std::vector<size_t> points_in_this_grid = grids_[i_row][i_col];
      if (points_in_this_grid.empty())
        continue;
      
      // TODO: if we shoudl deep check the points in this grid is in search_bounds
      for (auto idx : points_in_this_grid) {
        indices.push_back(idx);
      }
    }
  }

  return indices;
}
}