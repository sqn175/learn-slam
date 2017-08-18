/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef COMMON_PARAMETERS_READER_H_
#define COMMON_PARAMETERS_READER_H_

#include <string>

namespace lslam {

class ParamtersReader {
public:
  ParamtersReader() {}
  ParamtersReader(std::string file_name);
  ~ParamtersReader();
  
  void Read(std::string file_name);
private:
  CameraParams camera_params_;
};
} // namespace LSLAM

#endif  //COMMON_PARAMETERS_READER_H_
