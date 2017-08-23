/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "frame.h"
#include "camera_measurement.h"

namespace lslam {

class Frontend{
  
public:
  // Constructor
  Frontend() ;
  ~Frontend() {
  }
  
  void AddCameraMeasurement(const CameraMeasurement camera_measurement);
private:
  
  bool is_initialized_;  // Is the camera pose initialised?
  
  CameraMeasurement camera_measurement_prev; // previous camera_measurement
  
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
