/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "camera_measurement.h"
#include "map.h"

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
  
  bool Initialize();
  
  void AddCameraMeasurement(std::shared_ptr<CameraMeasurement> camera_measurement_current);
  
  
  
private:
  
  Map map_; 
  
  FrontEndState state_;  // frontend state
  
  std::shared_ptr<CameraMeasurement> camera_measurement_current_; // current camera_measurement
  
  std::shared_ptr<CameraMeasurement> camera_measurement_prev_; // previous camera_measurement
  
  // Initial data association to create initial map when we have a initial camera pose
  void DataAssociation();
  
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
