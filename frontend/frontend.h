/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <glog/logging.h>

#include "camera_measurement.h"
#include "map.h"
#include "pinhole_camera.h"
#include "parameters_reader.h"

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
  
  void init(const ParametersReader&);
  
  void Process(std::shared_ptr<CameraMeasurement> camera_measurement_current);
  
  std::shared_ptr<Map> map() const;
  
  // Accessors
  PinholeCamera camera_model() const;
  // Setters
  void set_camera_model(std::shared_ptr<PinholeCamera> camera_model);
private:
  
  Map map_; 

  // Camera model
  std::shared_ptr<PinholeCamera> camera_model_;

  // ORB extractor to detect and describe features
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  
  FrontEndState state_;  // frontend state
  
  std::shared_ptr<CameraMeasurement> camera_measurement_current_; // current camera_measurement
  
  std::shared_ptr<CameraMeasurement> camera_measurement_prev_; // previous camera_measurement
  
  bool DataAssociationInitialize();
  // Initial data association to create initial map when we have a initial camera pose
  void DataAssociation();
  
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
