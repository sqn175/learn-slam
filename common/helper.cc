/*
 * Author: ShiQin
 * Date: 2017-08-31
 */

#include "helper.h"

namespace lslam {

std::string MatType2Str(int type) {
  std::string r;

  unsigned char depth = type & CV_MAT_DEPTH_MASK;
  unsigned char chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

} // namespace lslam
