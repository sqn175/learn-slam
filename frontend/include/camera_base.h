/*
 * @Author: Shi Qin (sqn175@gmail.com) 
 * @Date: 2017-07-06 17:14:49 
 * @Last Modified by: ShiQin (sqn175@gmail.com)
 * @Last Modified time: 2017-07-07 10:58:42
 */

#ifndef FRONTEND_CAMERA_BASE_H_
#define FRONTEND_CAMERA_BASE_H_

#include <Eigen/Core>

namespace learnslam
{
class CameraBase
{
public:
  inline CameraBase() { image_width_ = 0; }
  // inline CameraBase(int image_width, int image_height, double frame_rate);

  inline virtual ~CameraBase() {}

  // the width of the image in pixels
  inline int image_width() const { return image_height_; }
  // the height of the image in pixels
  inline int image_height() const { return image_height_; }

  inline void set_image_width(int image_width)   { image_width_ = image_width; }
  inline void set_image_height(int image_height) { image_height_ = image_height; }

  virtual void intrinsics(Eigen::VectorXd &intrinsics) const = 0;

protected:
  int image_width_;   // image width in pixels
  int image_height_;  // image height in pixels
  double frame_rate_; // frame rate in Hz
};

} // namespace learnslam

#endif // FRONTEND_CAMERA_BASE_H_
