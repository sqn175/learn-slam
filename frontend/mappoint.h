/*
 * Author: ShiQin
 * Date: 2017-08-24
 */

#ifndef FRONTEND_LANDMARK_H
#define FRONTEND_LANDMARK_H

#include <memory>
#include <map>
#include <mutex>
#include <atomic>

#include <glog/logging.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "pinhole_camera.h"

namespace lslam {

class Frame;
class KeyFrame;

/**
 * @brief Thread-safe MapPoint class
 * 
 */
class MapPoint {
public:
  // Default Constructor
  MapPoint(const cv::Mat& pt_world, std::shared_ptr<KeyFrame> ref_kf);
  // Forbid copy constructor 
  MapPoint(const MapPoint&) = delete;
  // Forbid assignment operator
  MapPoint& operator=(const MapPoint&) = delete;

  ~MapPoint();

  // Accessors
  unsigned long id() const {
    return id_;
  }
  unsigned long created_by_keyframe_id() const {
    return created_by_keyframe_id_;
  }

  // Thread-safe accessors
  cv::Mat pt_world() const;
  cv::Mat descriptors() const;
  std::map<std::shared_ptr<KeyFrame>, size_t> observations() const;
  cv::Mat normal_vector() const;
  std::shared_ptr<KeyFrame> ref_keyframe() const;
  double max_distance() const;
  double min_distance() const;
  int cnt_projected() const;
  int cnt_tracked() const;

  size_t SizeOfObs() const;

  // Thread-safe mutators
  bool AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index);
  size_t EraseObservation(std::shared_ptr<KeyFrame> keyframe);
  // Compute the best descriptor associated to this landmark
  void SetDescriptors();
  // Should be called after:
  // 1. set_pt_world() called to change this mappoint's 3d position;
  // 2. ref_keyframe is modified;
  // 3. observed keyframes are modified.
  void SetNormalAndDepth();

  // Utility functions
  int PredictOctaveInFrame(std::shared_ptr<Frame> frame, const double& dist);
  // Check if this map is in the frustum of the camera
  bool IsProjectable(std::shared_ptr<Frame> frame, cv::Mat& uv, int& scale, double& view_cosine);
  
  // TODO: wrap two function into one
  bool IsObservedByKeyFrame(std::shared_ptr<KeyFrame> keyframe);
  // -1, for not found
  int GetIndexInKeyFrame(std::shared_ptr<KeyFrame> keyframe);

  bool ReplaceWith(std::shared_ptr<MapPoint> mp);

  // After call this functon, this mappoint should be erased from map by calling map->EraseMappoint()
  void SetBadFlag();

  void set_pt_world(const cv::Mat&);

  bool is_bad() const;


  void IncreaseCntProjected(int n = 1);
  void IncreaseCntTracked(int n = 1);
  double TrackedRatio();

  
private:
  unsigned long id_; // ID of the map point
  unsigned long created_by_keyframe_id_; // The reference keyframe id when creating this mappoint
  cv::Mat pt_world_; // 3d coordinate of the map point in the world map
  cv::Mat descriptors_; // Best descriptors of this point

  /// If the mappoint have a corresponding keypoint in KeyFrame A at index i, then pair {A,i} is an observation
  std::map<std::shared_ptr<KeyFrame>, size_t> observations_;  

  cv::Mat normal_vector_; // Viewing direction

  // Reference keyframe, from which the mappoint is created(by triangulation)
  std::shared_ptr<KeyFrame> ref_keyframe_;
  // scale invariance distance
  double max_distance_; 
  double min_distance_;

  // If a new mappoint is added to map, we should evaluate if we add it properly 
  // by record the count it can be projected to a frame and has been matched to a frame in the Track thread
  int cnt_projected_;
  int cnt_tracked_;

  // If is bad, the allocated memory will be deleted after the smart pointer's count decreases to 0.
  std::atomic<bool> is_bad_; ///< Indicate that this mappoint is invalid.
  
  // Provide concurrency on observations_, descriptors_, ref_keyframe_, cnt_projected_, cnt_tracked_
  mutable std::mutex mutex_; 
  // Provide concurrency on pt_world_, normal_vector_, max_distance_, min_distance_,
  mutable std::mutex mutex2_;
  // TODO: consider if we should use a mutex for every data
public:
  static std::mutex mGlobalMutex;
  // test, track the lifetime of a specific mappoint
};

} // namespace lslam

#endif // FRONTEND_LANDMARK_H
