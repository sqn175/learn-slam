/*
 * Author: ShiQin
 * Date: 2017-08-29
 */

#include "mappoint.h"

#include <glog/logging.h>

#include "frame.h"
#include "map.h"
#include "keyframe.h"
#include <limits>

namespace lslam {

std::mutex MapPoint::mGlobalMutex;
// test
int DEBUG_ID = -1;

/**
 * @brief Default constructor of MapPoint. In monocular case, a mappoint is created by 
 *        triangulation If we find a match between two keyframes.
 * 
 * @param pt_world The 3D world coodinate.
 * @param ref_kf Reference keyframe which triangulate the mappoint.
 * 
 */
MapPoint::MapPoint(const cv::Mat& pt_world, std::shared_ptr<KeyFrame> ref_kf, const std::shared_ptr<Map>& map)
  : created_by_keyframe_id_(ref_kf->id())
  , is_bad_(false)
  , pt_world_(pt_world) 
  , ref_keyframe_(ref_kf)
  , cnt_projected_(1)
  , cnt_tracked_(1)
  , map_(map) {

  // Assign a global unique id 
  static unsigned long unique_id = 0;
  id_ = unique_id++;

  DLOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " was created.";
}

MapPoint::~MapPoint() {
  LOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " was deleted.";
}

cv::Mat MapPoint::pt_world() const {
  std::unique_lock<std::mutex> lock(mutex2_);
  return pt_world_.clone();
}

cv::Mat MapPoint::descriptors() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return descriptors_.clone();
}

/**
 * @brief Observations Accessor.
 * 
 * @return std::map<std::shared_ptr<KeyFrame>, size_t> Observations.
 */
std::map<std::shared_ptr<KeyFrame>, size_t> MapPoint::observations() const {
  // TODO: using shared_lock, read-write lock
  std::unique_lock<std::mutex> lock(mutex_);
  return observations_;
}

cv::Mat MapPoint::normal_vector() const {
  std::unique_lock<std::mutex> lock(mutex2_);
  return normal_vector_.clone();
}

std::shared_ptr<KeyFrame> MapPoint::ref_keyframe() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return ref_keyframe_;
}

double MapPoint::max_distance() const {
  std::unique_lock<std::mutex> lock(mutex2_);
  return max_distance_;
}

double MapPoint::min_distance() const {
  std::unique_lock<std::mutex> lock(mutex2_);
  return min_distance_;
}

int MapPoint::cnt_projected() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return cnt_projected_;
}

int MapPoint::cnt_tracked() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return cnt_tracked_;
}


/**
 * @brief Add an observation to mappoint.
 * 
 * @param keyframe Keyframe observing this mappoint.
 * @param keypoint_index Keypoint index in the keyframe corresponding to the mappoint.
 * @return true If inserted successfully.
 * @return false Keyframe already exists, we do not insert.
 * 
 * An observation is added when:
 * - In mapper thread, a new keyframe was created from frame, 
 *   the corresponding mappoints in the frame were observed by the keyframe;
 * - In mapper thread, a new mappoint is triangulated by two matched keyframes,
 *   the mappoint was observed by the two keyframes;
 * - In mapper thread, Mapper#FuseAndAssociateMapPoints called, mappoint A observed by keyframe A
 *   may be replaced by another mappoint B, so mappoint B is then observed by keyframe A. 
 *   Or mappoint B observed by keyframe B has a corresponding keypoint in keyframe A, so mappoint B
 *   is then observed by keyframe A. 
 */
bool MapPoint::AddObservation(std::shared_ptr<KeyFrame> keyframe, int keypoint_index) {
  LOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " add observation{kf_id, kp_idx}:" 
                            << keyframe->id() << ", " << keypoint_index;
  std::unique_lock<std::mutex> lock(mutex_);
  auto ret = observations_.insert({keyframe, keypoint_index});
  return ret.second;
}

/**
 * @brief Erase an observation.
 * 
 * @param keyframe Keyframe observing this mappoint.
 * @return size_t The number of observations erased, as the keyframe is unique, 
 *         return value is 1 or 0.
 * 
 * An observation is erased when:
 * - A keyframe is deleted;
 * - In mapper thread, LocalBundleAdjustment() set the observation as outlier
 * 
 * @note After erasation, we NEED to check this mappoint has at least two observations
 *       to be a valid mappoint, otherwise delete this mappoint.
 */
size_t MapPoint::EraseObservation(std::shared_ptr<KeyFrame> keyframe) {
  LOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " erase observation: kf_id = "
                            << keyframe->id();

  std::unique_lock<std::mutex> lock(mutex_);
  size_t i = observations_.erase(keyframe);
  int obs_size = observations_.size();
  lock.unlock();
  
  if (obs_size <= 2) {
    SetBadFlag();
    return i;
  } 

  std::unique_lock<std::mutex> lock1(mutex_);
  if (ref_keyframe_ == keyframe) {
    // TODO: this is a casual reference, can we select a better one?
    // TODO: check observations_ have at least one element
    ref_keyframe_ = observations_.begin()->first;
  }
  return i;
}

/**
 * @brief The size of observations.
 * 
 * @return size_t Size.
 */
size_t MapPoint::SizeOfObs() const {
  std::unique_lock<std::mutex> lock(mutex_);
  return observations_.size();
}

/**
 * @brief Compute the best descriptor of this mappoint according to observations.
 * 
 * @note Need to be called after observation modifications.
 * 
 */
void MapPoint::SetDescriptors() {
  if (is_bad_)
    return;
  // Thread-safe copy
  auto obs = observations();
  
  auto size = obs.size();
  // We retrieve all observed descriptors
  if (size == 0) 
    return;

  std::vector<cv::Mat> all_descriptors;
  all_descriptors.reserve(size);
  for (auto& item : obs) {
    all_descriptors.push_back(item.first->descriptors().row(item.second));
  }
  
  // Compute distances between them
  double distances[size][size];
  for (size_t i = 0; i < size; ++i) {
    distances[i][i] = 0;
    for (size_t j = i+1; j < size; ++j) {
      int dist_ij = cv::norm(all_descriptors[i], all_descriptors[j], cv::NORM_HAMMING);
      distances[i][j] = dist_ij;
      distances[j][i] = dist_ij;
    }
  }
  
  // We take the descriptor with least median distance to the rest
  double best_median = std::numeric_limits<double>::max();
  int best_idx = 0;
  for (size_t i = 0; i < size; ++i) {
    // For descriptor i, we get all the distances to other descriptors
    std::vector<double> dists(distances[i], distances[i] + size);
    std::sort(dists.begin(), dists.end());
    double median = dists[0.5*(size-1)];
    if (median < best_median) {
      best_median = median;
      best_idx = i;
    }
  }
  
  std::unique_lock<std::mutex> lock(mutex_);
  descriptors_ = all_descriptors[best_idx].clone();
}


// Distance invariance 
//              ____
// Nearer      /____\     level:n-1 --> dmin
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
void MapPoint::SetNormalAndDepth() {
  // Thread-safe copy
  auto obs = observations();
  cv::Mat pt_w = pt_world();

  cv::Mat normal = cv::Mat::zeros(3,1,CV_64F);
  int i = 0;
  for (auto& ob : obs) {
    auto keyframe = ob.first;
    cv::Mat n = pt_w - keyframe->o_w();
    normal = normal + n/cv::norm(n);
    ++i;
  }

  auto ref_kf = ref_keyframe();

  cv::Mat n_ref = pt_w - ref_kf->o_w();
  double dist = cv::norm(n_ref);
  int level = ref_kf->undistorted_kp(observations_[ref_kf]).octave;
  auto level_scale_factors = ref_kf->orb_extractor()->GetScaleFactors();
  double level_scale_factor = level_scale_factors[level];
  int n_level = ref_kf->orb_extractor()->GetLevels();

  // Thread-safe assignment
  std::unique_lock<std::mutex> lock(mutex2_);
  normal_vector_ = normal / i;
  max_distance_ = dist * level_scale_factor;
  min_distance_ = max_distance_ / level_scale_factors[n_level-1];
}

int MapPoint::PredictOctaveInFrame(std::shared_ptr<Frame> frame, const double& dist) {
  double log_scale_factor = frame->orb_extractor()->GetScaleFactor();

  // Thread-safe copy
  double max_dist = max_distance();

  int octave = std::ceil(std::log(max_dist/dist) / std::log(log_scale_factor));
  if (octave < 0) {
    octave = 0;
  } else if (octave >= frame->orb_extractor()->GetLevels()) {
    octave = frame->orb_extractor()->GetLevels() - 1;
  }
  return octave;
}

bool MapPoint::IsProjectable(std::shared_ptr<Frame> frame, cv::Mat& p_uv, int& octave, double& view_cosine) {
  CHECK(frame) << "frame is Null.";
  auto camera_model = frame->camera_model();
  CHECK(camera_model) << "camera_model is Null.";

  p_uv.release();

  if (!frame->T_cw().data) {
    LOG(ERROR) << "The pose of frame is NOT set.";
    return false;
  }
  // 3D Landmark point in camera coordinates
  cv::Mat pt_w = pt_world();
  cv::Mat p_c = frame->Project(pt_w);
  
  // Check positive depth
  const double pz_c = p_c.at<double>(2);
  if (pz_c <= 0.0) 
    return false;

  // Project to image coordinate and check it is inside
  p_uv = camera_model->Project(p_c);
  double u = p_uv.at<double>(0);
  double v = p_uv.at<double>(1);
  cv::Mat img_bounds = camera_model->image_bounds();
  double min_u = img_bounds.at<double>(0);
  double min_v = img_bounds.at<double>(1);
  double max_u = img_bounds.at<double>(2);
  double max_v = img_bounds.at<double>(3);
  if (u < min_u || u > max_u || v < min_v || v > max_v)
    return false;

  // Check distance is in the scale invariance region
  cv::Mat n = pt_w - frame->o_w();
  double dist = cv::norm(n);
  // TUNE: 
  if (dist < 0.8*min_distance() || dist > 1.2*max_distance())
    return false;

  // Check viewing angle > 60 degree
  view_cosine = n.dot(normal_vector())/dist;
  // TUNE: view_cosine_limit 0.5
  if (view_cosine < 0.5) 
    return false;

  octave = PredictOctaveInFrame(frame, dist);

  return true;
}

bool MapPoint::IsObservedByKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_);
  return observations_.count(keyframe);
}

int MapPoint::GetIndexInKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (observations_.count(keyframe)) 
    return observations_[keyframe];
  else  
    return -1;
}

bool MapPoint::ReplaceWith(std::shared_ptr<MapPoint> mp) {
  LOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " was replaced with mappoint id = " << mp->id();
  if (mp->id() == id_) 
    return false;

  is_bad_ = true;
  auto obs = observations();
  // This mappoint will be erased, so we need to update the 
  // keyframes observing this mappoint
  for (auto& ob : obs) {
    auto kf = ob.first;
    if (mp->IsObservedByKeyFrame(kf)) {
      // mp is already associated to this observing keyframe, 
      // we simply erase the association of this mappoint and the observing keyframe
      kf->EraseMapPoint(ob.second);
    } else {
      kf->set_mappoint(ob.second, mp);
      mp->AddObservation(kf, ob.second);
    }
  }
  
  mp->IncreaseCntProjected(cnt_projected());
  mp->IncreaseCntTracked(cnt_tracked());
  mp->SetDescriptors();
  // TODO: need to update depth? 

  // Erase this mappoint in the map
  map_->EraseMapPoint(shared_from_this());

  return true;
}

void MapPoint::SetBadFlag() {
  LOG_IF(INFO, id_==DEBUG_ID) << "Mappoint " << DEBUG_ID << " was set as bad.";
  is_bad_ = true;
  auto obs = observations();
  for (auto& ob : obs) {
    auto kf = ob.first;
    // Update association
    kf->EraseMapPoint(ob.second);
  }

  // erase this mappoint in the map
  map_->EraseMapPoint(shared_from_this());
}

void MapPoint::set_pt_world(const cv::Mat& pt_world) {
  CHECK(pt_world.cols == 1 && pt_world.rows == 3) << "invalid pt_world dimension.";
  std::unique_lock<std::mutex> lock2(mGlobalMutex); // TODO: why we need this mutex?
  std::unique_lock<std::mutex> lock(mutex2_);
  pt_world.copyTo(pt_world_);
}

bool MapPoint::is_bad() const {
  return is_bad_;
}

void MapPoint::IncreaseCntProjected(int n) {
  std::unique_lock<std::mutex> lock(mutex_);
  cnt_projected_ += n;
}
void MapPoint::IncreaseCntTracked(int n) {
  std::unique_lock<std::mutex> lock(mutex_);
  cnt_tracked_ += n;
}

double MapPoint::TrackedRatio() {
  std::unique_lock<std::mutex> lock(mutex_);
  CHECK(cnt_projected_ != 0);
  return (double)cnt_tracked_ / cnt_projected_;
}


} // namespace lslam
