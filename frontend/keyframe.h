/*
 * Author: ShiQin
 * Date: 2017-08-25
 */

#ifndef FRONTEND_KEYFRAME_H
#define FRONTEND_KEYFRAME_H

#include "camera_measurement.h"

namespace lslam {

class KeyFrame : public CameraMeasurement {
public:
  KeyFrame(CameraMeasurement&);
  ~KeyFrame();
  
};

} // namespace lslam


#endif // FRONTEND_KEYFRAME_H
