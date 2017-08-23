/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef COMMON_PARAMETERS_READER_H_
#define COMMON_PARAMETERS_READER_H_

#include <string>

#include "Parameters.h"

namespace lslam {

class ParamtersReader {
public:
  ParamtersReader() {}
  ParamtersReader(const std::string &file_name);

  ~ParamtersReader() {}
  
  void Read(const std::string &file_name);
  
  // Accessors
  PinholeCameraParameters pinholecamera_params() const;
  
protected:
  PinholeCameraParameters pinholecamera_params_; // Pinhole camera parameters.
};
} // namespace LSLAM

#endif  //COMMON_PARAMETERS_READER_H_
