/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_FTONTEND_H_
#define FRONTEND_FRONTEND_H_

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <glog/logging.h>

#include "../3rdparty/ORB_SLAM2_modified/ORBextractor.h"
#include "../3rdparty/DBoW2/DBoW2/FORB.h"
#include "../3rdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include "pinhole_camera.h"
#include "parameters_reader.h"
#include "guided_matcher.h"
#include "cv.h"
#include "helper.h"

namespace lslam {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
class Map;
class Frame;
class KeyFrame;

class Frontend{
public:
  enum FrontEndState {
    kNotInitialized = 0, // Not initialized
    kInitialized,    // Just initialized, we use this state to publish two initial keyframes
    kTracking,       // Tracking
  };
  
public:
  // Constructor
  Frontend();

  Frontend(std::shared_ptr<Map> map,
           std::shared_ptr<PinholeCamera> camera_model,
           std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor,
           std::shared_ptr<ORBVocabulary> orb_voc,
           std::shared_ptr<GuidedMatcher> guided_matcher);

  ~Frontend() {
  }
  
  void Process(cv::Mat image, double timestamp);
  //void Process(std::shared_ptr<Frame> camera_measurement_current);
  
  bool FrameIsKeyFrame();
  void CreateKeyFrame();

  // Accessors
  PinholeCamera camera_model() const;
  FrontEndState state() const { return state_; }
  // TODO: redesign this interference, frontend only output frame, and decide if this is a keyframe,
  // We construct a keyframe in the mapper thread.
  std::shared_ptr<KeyFrame> cur_keyframe() const { return cur_keyframe_; }
  std::shared_ptr<KeyFrame> init_keyframe() const { return init_keyframe_; }

  std::shared_ptr<Frame> cur_frame() const { return cur_frame_; }

  cv::Mat image() const { return image_.clone(); }
  // Setters
  void set_map(std::shared_ptr<Map> map);
  void set_camera_model(std::shared_ptr<PinholeCamera> camera_model);

private:
  // Initial data association to create initial 3D map when we have a initial camera pose
  bool DataAssociationBootstrap();
  // Process incoming frames as quickly as possible
  void DataAssociation();

  // Initial pose estimation from previous frame
  bool TrackToLastFrame();
  bool TrackToLastKeyFrame();

  // 
  bool TrackToLocalMap();

  // Return true if current frame is a keyframe
  bool AsKeyFrame();
  
private:
  
  std::shared_ptr<Map> map_; 

  // Camera model
  std::shared_ptr<PinholeCamera> camera_model_;
  // ORB extractor to detect and describe features
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  // ORB Vocabulary
  std::shared_ptr<ORBVocabulary> orb_voc_;
  // Guided matcher
  std::shared_ptr<GuidedMatcher> guided_matcher_;
  
  FrontEndState state_;  // frontend state
  
  // TODO: redesign this interference, do we need to handle keyframe things in this class?
  std::shared_ptr<KeyFrame> cur_keyframe_;
  std::shared_ptr<KeyFrame> init_keyframe_;

  std::shared_ptr<Frame> cur_frame_; // current camera_measurement
  
  std::shared_ptr<Frame> last_frame_; // previous camera_measurement

  std::shared_ptr<Frame> init_frame_; // Initial frame used for initialization
  
  // Visualized Data
  cv::Mat image_;

  //
  std::shared_ptr<KeyFrame> reference_keyframe_;
  // Keyframe insertion decision parameters, TODO: wrap this out of Frontend class
  int max_keyframe_interval_;
  int min_keyframe_interval_;
  size_t last_frame_id_as_kf_;
  int n_match_to_localmap_;
  // Associated local 
};

} // namespace lslam

#endif // FRONTEND_FRONTEND_H_
