/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "camera_measurement.h"

namespace lslam {

class Frontend{

public:
  enum class FrontEndState {
    kNotReady = 0,   // no camera measurement
    kNotInitialized, // not initialized
    kInitialized,    // initialized
  };
  
public:
  // Constructor
  Frontend();
  ~Frontend() {
  }
  
  
  
  void AddCameraMeasurement(CameraMeasurement& camera_measurement);
  
private:
  FrontEndState state;  // frontend state
  
  CameraMeasurement camera_measurement_prev; // previous camera_measurement
  
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
