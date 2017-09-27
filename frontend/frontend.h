/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <glog/logging.h>

#include "frame.h"
#include "map.h"
#include "pinhole_camera.h"
#include "parameters_reader.h"
#include "guided_matcher.h"

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
  
  void Process(std::shared_ptr<Frame> camera_measurement_current);
  
  // Accessors
  PinholeCamera camera_model() const;
  // Setters
  void set_map(std::shared_ptr<Map> map);
  void set_camera_model(std::shared_ptr<PinholeCamera> camera_model);

private:
  // Initial data association to create initial 3D map when we have a initial camera pose
  bool DataAssociationBootstrap();
  // Process incoming frames as quickly as possible
  void DataAssociation();
  
private:
  
  std::shared_ptr<Map> map_; 

  // Camera model
  std::shared_ptr<PinholeCamera> camera_model_;

  // ORB extractor to detect and describe features
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  
  FrontEndState state_;  // frontend state
  
  std::shared_ptr<Frame> cur_frame_; // current camera_measurement
  
  std::shared_ptr<Frame> last_frame_; // previous camera_measurement
  
  // Guided matcher
  GuidedMatcher guided_matcher_;
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
