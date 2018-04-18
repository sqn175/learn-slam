/*
 * @Author: Shi Qin 
 * @Date: 2017-09-26 15:24:05 
 * @Last Modified by: Shi Qin
 * @Last Modified time: 2017-09-26 17:13:00
 */

#include "range_searcher.h"

namespace lslam {

RangeSearcher::RangeSearcher(const std::vector<cv::KeyPoint>& points, const cv::Mat& bounds, int n_points_per_grid) {
  BuildGrids(points, bounds, n_points_per_grid);
}

void RangeSearcher::BuildGrids(const std::vector<cv::KeyPoint>& points, const cv::Mat& bounds, int n_points_per_grid) {
  int n_points = points.size();
  int n_grids = std::ceil((double)n_points/(double)n_points_per_grid);
  min_x_ = bounds.at<double>(0,0);
  min_y_ = bounds.at<double>(0,1);
  max_x_ = bounds.at<double>(1,0);
  max_y_ = bounds.at<double>(1,1);
  // height / width ratio
  double ratio = (max_x_ - min_x_) / (max_y_ - min_y_);
  grids_rows_ = std::ceil(std::sqrt((double)n_grids / ratio));
  grids_cols_ = std::ceil((double)n_grids / (double)grids_rows_);
  n_grids = grids_cols_*grids_rows_;
  // TODO: debug grid_size_inv_ = (double)grids_rows_ / (max_u - min_u)
  grid_size_inv_ = (double)grids_rows_ / (max_y_ - min_y_);

  // construct grids
  grids_ = std::vector<std::vector<std::vector<size_t>>>(
                grids_rows_, std::vector<std::vector<size_t>>
                (grids_cols_, std::vector<size_t>()));
  // Assign points to grids
  for (size_t i = 0; i < n_points; ++i) {
    const cv::KeyPoint &kp = points[i];
    int grid_col = std::round( (kp.pt.x-min_x_)*grid_size_inv_ );
    int grid_row = std::round( (kp.pt.y-min_y_)*grid_size_inv_ );
    if (grid_col < 0 || grid_col >= grids_cols_ || grid_row < 0 || grid_row >= grids_rows_) {
      LOG(INFO) << "Assign feature to grids out of range.";
      continue;
    }
    grids_[grid_row][grid_col].push_back(i);
  }
}

std::vector<size_t> RangeSearcher::PointsInRange(const cv::Mat& search_bounds) {
  CHECK(search_bounds.cols == 2 && search_bounds.rows == 2) << "invalid search_bounds.";
  std::vector<size_t> indices;
  double min_x_search = search_bounds.at<double>(0,0);
  double min_y_search = search_bounds.at<double>(0,1);
  double max_x_search = search_bounds.at<double>(1,0);
  double max_y_search = search_bounds.at<double>(1,1);
  
  // TODO: check if assigned correctly
  int min_grid_col = std::max(0, (int)std::floor((min_x_search - min_x_)*grid_size_inv_));
  if (min_grid_col >= grids_cols_) return indices;
  int min_grid_row = std::max(0, (int)std::floor((min_y_search - min_y_)*grid_size_inv_));
  if (min_grid_row >= grids_rows_) return indices;
  int max_grid_col = std::min(grids_cols_-1, (int)std::ceil((max_x_search - min_x_)*grid_size_inv_));
  if (max_grid_col < 0) return indices;
  int max_grid_row = std::min(grids_rows_-1, (int)std::ceil((max_y_search - min_y_)*grid_size_inv_));
  if (max_grid_row < 0) return indices;

  for (int i_row = min_grid_row; i_row <= max_grid_row; ++i_row) {
    for (int i_col = min_grid_col; i_col <= max_grid_col; ++i_col) {
      const std::vector<size_t> points_in_this_grid = grids_[i_row][i_col];
      if (points_in_this_grid.empty())
        continue;
      
      // TODO: if we should deep check the points in this grid is in search_bounds
      for (auto idx : points_in_this_grid) {
        indices.push_back(idx);
      }
    }
  }

  return indices;
}
}