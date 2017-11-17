/*
 * Author: ShiQin
 * Date: 2017-08-21
 */

#ifndef FRONTEND_CAMERA_MEASUREMENT_H_
#define FRONTEND_CAMERA_MEASUREMENT_H_

#include <vector>
#include <memory>
#include <set>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "../3rdparty/DBoW2/DBoW2/FORB.h"
#include "../3rdparty/DBoW2/DBoW2/TemplatedVocabulary.h"
#include "../3rdparty/DBoW2/DBoW2/BowVector.h"
#include "../3rdparty/DBoW2/DBoW2/FeatureVector.h"

#include <ORBextractor.h>
#include "pinhole_camera.h"
#include "range_searcher.h"

namespace lslam {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

class MapPoint;
class KeyFrame;

class Frame{ 
public:
  
  // No default constructor
  Frame();
  
  Frame(const cv::Mat& image,double timestamp, 
        std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor,
        std::shared_ptr<PinholeCamera> camera_model,
        std::shared_ptr<ORBVocabulary> orb_voc);
  
  // Copy constructor
  Frame(const Frame& frame);
  Frame& operator=(const Frame&) = delete;
  
  ~Frame() {
  }

  // Accessors
  virtual unsigned long id() const;
  std::vector<cv::KeyPoint> keypoints() const;
  const cv::KeyPoint& keypoint(size_t idx) const;
  const std::vector<cv::KeyPoint>& undistorted_kps() const;
  const cv::KeyPoint& undistorted_kp(size_t idx) const;
  const cv::Mat& descriptors() const;
  virtual std::vector<std::shared_ptr<MapPoint>> mappoints() const;
  std::shared_ptr<RangeSearcher> range_searcher() const;
  virtual std::shared_ptr<MapPoint> mappoint(size_t idx) const; 
  std::shared_ptr<PinholeCamera> camera_model() const;
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor() const;
  bool outlier(size_t) const;
  const DBoW2::BowVector& bow_vector() const;
  const DBoW2::FeatureVector& feature_vector() const;
  std::set<std::shared_ptr<MapPoint>> connected_mappoints() const;

  virtual void SetPose(const cv::Mat& T_cw);
  virtual void set_mappoint(size_t idx, std::shared_ptr<MapPoint> mappoint);
  void set_outlier(size_t idx, bool flag);

  virtual cv::Mat T_cw() const;
  virtual cv::Mat T_wc() const;
  virtual cv::Mat o_w() const;
  virtual cv::Mat R_cw() const;
  virtual cv::Mat t_cw() const;

  // Project world coordinate 3d point to image coordinate 3d point
  cv::Mat Project(const cv::Mat pt3d_w);

  cv::Mat KRtProject(const cv::Mat& p3d);

  void ComputeBoW();
  // 
  virtual void SetConnectedKeyFrames();

  virtual void EraseMapPoint(const size_t& idx);
    
protected:
  // === Raw data ===
  double timestamp_; ///< timestamp of this frame
  unsigned long id_; // id of this frame
  // === Raw data ===

  // The camera geometry
  std::shared_ptr<PinholeCamera> camera_model_;
  // Feature detector and descriptor extractor
  std::shared_ptr<ORB_SLAM2::ORBextractor> orb_extractor_;
  // ORB Vocabulary
  std::shared_ptr<ORBVocabulary> orb_voc_;

  // === Data evidence ===
  std::vector<cv::KeyPoint> keypoints_; // the keypoints as OpenCV's struct
  std::vector<cv::KeyPoint> undistorted_kps_; // The undistorted keypoints
  cv::Mat descriptors_; // the descriptors as OpenCV's matrix 
  DBoW2::BowVector bow_vector_; // BoW
  DBoW2::FeatureVector feature_vector_; // Bow
  // === Data evidence ===

  // === State ===
  // MapPoints associated to keypoints, NULL for no association
  std::vector<std::shared_ptr<MapPoint>> mappoints_;
  // Flag to identify outlier associations
  std::vector<bool> outliers_; 
  // Camera Pose, cw -> world coordinate to camera coordinate
  cv::Mat T_cw_; // transformation matrix
  cv::Mat R_cw_; // rotation matrix
  cv::Mat t_cw_; // translation vector
  cv::Mat o_w_;  // camera original in world coordinate
  // Camera inverse pose, wc -> camera coordinate to world coordinate
  cv::Mat T_wc_; // inverse transformation matrix

  // === State ===

  // Connected Keyframes and mappoints(Local map), used for local track and visualization
  std::map<std::shared_ptr<KeyFrame>, int> connected_keyframes_weights_;// Covisibility
  std::set<std::shared_ptr<MapPoint>> local_mappoints_; 

  // Build feature points gridding range searcher
  std::shared_ptr<RangeSearcher> range_searcher_;

};

} // namespace lslam

#endif // FRONTEND_CAMERA_MEASUREMENT_H_
