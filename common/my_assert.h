/*
 * Author: ShiQin
 * Date: 2017-08-11
 */

#ifndef COMMON_MY_ASSERT_H_
#define COMMON_MY_ASSERT_H_

#include <exception>
#include <string>
#include <sstream>
#include <iostream>

namespace lslam {

class AssertFailureException : public std::exception {
public:
  explicit AssertFailureException(const std::string& expression, const std::string file, const std::string func, int line, const std::string& message)
    : expression_(expression)
    , file_(file)
    , func_(func)
    , line_(line)
    , message_(message) {
    
    std::ostringstream assert_stringstream;
    
    assert_stringstream << "Assertion: " << expression << " failed: " <<\
                           file << ": line:" << line << ": function:" << func << "(): " <<\
                           message;
    
    error_ = assert_stringstream.str();
  }
  
  virtual ~AssertFailureException() throw() {}
  
  virtual const char* what() const throw(){
    return error_.c_str();
  }
  
protected:
  std::string expression_;
  std::string file_;
  std::string func_;
  int line_;
  std::string message_;
  std::string error_;            
};

} // namespace LSLAM

// Assert macro, if assert expresstion failed, throw exception
#define ASLAM_ASSERT_TRUE(expression, message) \
  if (!(expression)) { \
    throw lslam::AssertFailureException(#expression, __FILE__, __FUNCTION__, __LINE__, message);\
  }

#endif // COMMON_MY_ASSERT_H_
