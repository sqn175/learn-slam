/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef COMMON_PARAMS_MAN_H_
#define COMMON_PARAMS_MAN_H_

#include <string>

#include "parameters.h"

namespace lslam {

class ParamsMan {
public:
  // Singleton design pattern
  static ParamsMan& Instance();

  void Read(const std::string &file_name);

public:
  PinholeCameraParameters pinholecamera_params; // Pinhole camera parameters.
  FeatureParameters feature_params; 
private:
  ParamsMan();
  ~ParamsMan();
};
} // namespace LSLAM

#endif  //COMMON_PARAMS_MAN_H_
