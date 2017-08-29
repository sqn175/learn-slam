/*
 * Author: ShiQin
 * Date: 2017-08-29
 */

#include "landmark.h"

namespace lslam {

Landmark::Landmark() {
}

Landmark::Landmark(unsigned int id, Eigen::Vector4d point_world) 
  : id_(id), point_world_(point_world) { }


} // namespace lslam
